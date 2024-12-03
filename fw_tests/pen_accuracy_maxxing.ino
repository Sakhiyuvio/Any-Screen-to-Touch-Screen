#include <SPI.h>
#include "DW1000Ranging.h"
#include <BleMouse.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Adafruit_LSM6DS.h>
#include <Adafruit_LSM6DSL.h>

#define PEN_UWB_ADDR "7D:00:22:EA:82:60:3B:9C" // unique addr
#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1"

// anchor short addresses
#define ANCHOR_ADDR_1 0x1782
#define ANCHOR_ADDR_2 0x1784

// ESP32-S3 SPI pin config - UWB
#define SPI_SCLK 12
#define SPI_MISO 13
#define SPI_MOSI 11
#define SPI_CS 10

// ESP32-S3 SPI pin config - IMU
#define IMU_SPI_SCLK 8
#define IMU_SPI_MISO 9
#define IMU_SPI_MOSI 7
#define IMU_SPI_CS 6
#define IMU_INT_1 18
#define IMU_INT_2 19

// ESP32-S3 pen-button pin
#define PEN_BUTTON 21

// data rate for pen comm
#define comm_data_rate 115200
#define comm_init_delay 1000

// gravitational constant
#define g 9.8

// BLE mouse vars
#define SCROLL_THRES 3000 // hold for 3 seconds to scroll
#define CLICK_THRES 2000

// samples for ranging
#define NUM_SAMPLES 50
#define ACC_THRES 2

// BLE setup
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool host_dev_connected = false;

unsigned long pen_button_start_time = 0;
bool is_pen_pressed = false;

const uint8_t RST_pin = 5;
const uint8_t CS_pin = 10;
const uint8_t INT_pin = 1;

// global SPI class and lsm6dsl instance for the IMU
SPIClass SPI2;
Adafruit_LSM6DSL imu_dsl;

// global variables for ranging storage - uwb
float prev_prev_range_uwb_1, prev_range_uwb_1, curr_range_uwb_1;
float prev_prev_range_uwb_2, prev_range_uwb_2, curr_range_uwb_2;
float range_uwb_1_buf[NUM_SAMPLES] = {0};
float range_uwb_2_buf[NUM_SAMPLES] = {0};
int uwb_1_buf_idx = 0;
int uwb_2_buf_idx = 0;
int count_idx_1 = 0;
int count_idx_2 = 0;
int ranging_flag = 0;
// float weight_factor_1 = 0; 
// float weight_factor_2 = 0; 
// float known_dist_1 = 0; // change to actual known_distance for scaling configuration 
// float known_dist_2 = 0; // change to actual known_distance for scaling configuration 
int set_flag_1 = 0; 
int set_flag_1_2 = 0; 
int set_flag_2 = 0; 
int set_flag_2_2 = 0; 

// global variables for imu
float acc_x, acc_y, acc_z;
float gyr_x, gyr_y;
float acc_roll_angle, acc_pitch_angle;
float gyr_roll_angle, gyr_pitch_angle;
float curr_pitch_angle;
float curr_roll_angle;
float trust_factor = 0.95;
float trust_factor_complement = 1 - trust_factor;

// time-keeping
float delta_t, delta_t_uwb;
unsigned long prev_time, prev_time_uwb;

// global variables to keep track of current screen coordinates
float curr_x, curr_y;
int cursor_x, cursor_y;
int prev_x, prev_y;

// hardcoded parameters pre-calibration
float pen_length = 9.8; // in cm
float screen_width = 0.475; //47.5 cm
float screen_height = 0.265; //26.5 cm
int res_x = 1920;
int res_y = 1080;

// bluetooth mouse instance
BleMouse bleMouse;

// BLE callback

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect (BLEServer* pServer) {
      host_dev_connected = true;
      Serial.println("Connected to GUI");
    }

    void onDisconnect (BLEServer* pServer) {
      host_dev_connected = false;
      Serial.println("Disconnected from GUI");
    }
};

void setup()
{
  Serial.begin(comm_data_rate);
  delay(comm_init_delay);

  // init IMU communication
  pinMode(IMU_INT_1, INPUT_PULLDOWN);
  pinMode(IMU_INT_2, INPUT_PULLDOWN);
  // Configure pins
  pinMode(IMU_SPI_CS, OUTPUT);
  digitalWrite(IMU_SPI_CS, HIGH);  // Initially HIGH
  SPI2.begin(IMU_SPI_SCLK, IMU_SPI_MISO, IMU_SPI_MOSI, IMU_SPI_CS);
  if (!imu_dsl.begin_SPI(IMU_SPI_CS, &SPI2)) {
    if (!imu_dsl.begin_SPI(IMU_SPI_CS, IMU_SPI_SCLK, IMU_SPI_MISO, IMU_SPI_MOSI)) {
      Serial.println("Failed to find LSM6DSL chip");
    }
  }
  acc_x = 0;
  acc_y = 0;
  acc_z = 0;
  gyr_x = 0;
  gyr_y = 0;
  curr_pitch_angle = 0.0;
  curr_roll_angle = 0.0;
  acc_roll_angle = 0.0;
  acc_pitch_angle = 0.0;
  gyr_roll_angle = 0.0;
  gyr_pitch_angle = 0.0;
  delta_t = 0.0;
  prev_time = millis();

  // init pen button
  pinMode(PEN_BUTTON, INPUT_PULLUP);
  // enabling internal pullup
  //    digitalWrite(PEN_BUTTON, HIGH);

  // init uwb communication
  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, SPI_CS);
  DW1000Ranging.initCommunication(RST_pin, CS_pin, INT_pin);
  // consider delaying
  prev_prev_range_uwb_1 = 0;
  prev_prev_range_uwb_2 = 0; 
  prev_range_uwb_1 = 0;
  prev_range_uwb_2 = 0;
  curr_range_uwb_1 = 0;
  curr_range_uwb_2 = 0;
  // handlers of getting data and connecting to other uwb transceivers
  DW1000Ranging.attachNewRange(ranging_handler);
  DW1000Ranging.attachNewDevice(new_dev_handler);
  DW1000Ranging.attachInactiveDevice(inactive_handler);
  // start DWM as tag
  DW1000Ranging.startAsTag(PEN_UWB_ADDR, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

  // TO DO: set up data post-processing to host device here, via Bluetooth BLE
  // TO DO: I think the cursor should be initialized during calibration process
  // set to 0 for now
  curr_x = 0.0;
  curr_y = 0.0;
  cursor_x = 0;
  cursor_y = 0;
  prev_x = 0;
  prev_y = 0;

  // BLE dev and HID init
  //    BLEDevice::init("ESP32-BLE");
  //    pServer = BLEDevice::createServer();
  //    pServer->setCallbacks(new MyServerCallbacks());
  //    // BLE service
  //    BLEService *pService = pServer->createService(SERVICE_UUID); // service uuid
  //    pCharacteristic = pService->createCharacteristic(
  //        CHARACTERISTIC_UUID, // char uuid
  //        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  //    );

  //    pService->start();
  //    pServer->getAdvertising()->start();
  //    Serial.println("BLE Waiting for GUI Connection...");
  Serial.println("Starting BLE Mouse");
  bleMouse.begin();
}

void loop() // Arduino IDE, continuous looping
{
  DW1000Ranging.loop(); // tag-anchor communication

  // set up ranging loop for the IMU here
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  imu_dsl.getEvent(&accel, &gyro, &temp);

  acc_x = accel.acceleration.x;
  acc_y = accel.acceleration.y;
  acc_z = accel.acceleration.z;
  gyr_x = gyro.gyro.x;
  gyr_y = gyro.gyro.y;

  // perform adaptive noise filtering on the IMU sensor (Optional for now)

  // update time variables
  delta_t = (millis() - prev_time) / 1000.0;

  // process the roll and pitch angle for configurating tilting through IMU sensor
  acc_roll_angle = atan2(acc_y / g, acc_z / g) * 180 / PI;
  acc_pitch_angle = atan2(acc_x / g, acc_z / g) * 180 / PI;
  gyr_roll_angle = gyr_x * delta_t;
  gyr_pitch_angle = gyr_y * delta_t;

  // complementary filter imp
  curr_roll_angle = (curr_roll_angle + gyr_roll_angle) * trust_factor + acc_roll_angle * trust_factor_complement;
  curr_pitch_angle = (curr_pitch_angle - gyr_pitch_angle) * trust_factor + acc_pitch_angle * trust_factor_complement;

  if (ranging_flag) {
      // at each time instance, we need to be prepared to send coordinates of the pen on the screen
      // pass in the tilting angle, as well as ranging parameters
      localization_algo(curr_roll_angle, curr_pitch_angle, curr_range_uwb_1, curr_range_uwb_2, pen_length, screen_width, screen_height, res_x, res_y);
   
      // process these data to replicate bluetooth HID        // Bluetooth HID emulation here, use the coordinates received after localization
      if (bleMouse.isConnected()) {
        // update per 20 milliseconds
        // if (millis() - prev_time > 20) {
        send_mouse_emulation();
        // CONSIDER DELAYS, use visual feedback to see if there are lags due to host device being overwhelmed
        // set prev_x and prev_y to be the current cursor values before updated
        prev_x = cursor_x;
        prev_y = cursor_y;
        // }
      }    
      ranging_flag = 0;
  }

  prev_time = millis();

  // add logic, set a timeout based protocol to send data
  // and signal to host device for monitoring
  //    if (host_dev_connected) {
  //        // send binary buffer
  //        uint8_t data[8]; // per byte
  //        memcpy(data, &curr_x, sizeof(float));
  //        memcpy(data + sizeof(float), &curr_y, sizeof(float));
  //        pCharacteristic->setValue(data, 8);
  //        pCharacteristic->notify();
  //    }

}

void localization_algo(float roll_angle, float pitch_angle, 
                       float range_uwb_1, float range_uwb_2, 
                       float pen_length, float screen_width, float screen_height, 
                       int res_x, int res_y) {
  // Convert angles to radians
  float pitch_angle_rad = pitch_angle * PI / 180;
  float roll_angle_rad = roll_angle * PI / 180;

  // More standard triangulation method
  // Assuming anchors are known positions (you'll need to replace with actual anchor coordinates)
  float anchor1_x = 0.0;  // X coordinate of first anchor
  float anchor1_y = 0.0;  // Y coordinate of first anchor
  float anchor2_x = screen_width;  // X coordinate of second anchor
  float anchor2_y = 0.0;  // Y coordinate of second anchor

  // Trilateration calculation (example - you may need to adjust)
  float A = -2 * anchor1_x + 2 * anchor2_x;
  float B = -2 * anchor1_y + 2 * anchor2_y;
  float C = pow(range_uwb_1, 2) - pow(range_uwb_2, 2) - 
            pow(anchor1_x, 2) + pow(anchor2_x, 2) - 
            pow(anchor1_y, 2) + pow(anchor2_y, 2);

  float D = -2 * anchor2_x + 2 * anchor1_x;
  float E = -2 * anchor2_y + 2 * anchor1_y;
  float F = pow(range_uwb_2, 2) - pow(range_uwb_1, 2) - 
            pow(anchor2_x, 2) + pow(anchor1_x, 2) - 
            pow(anchor2_y, 2) + pow(anchor1_y, 2);

  // Calculate x and y
  float curr_x = (C*E - F*B) / (A*E - D*B);
  float curr_y = (C*D - F*A) / (B*D - E*A);

  // Apply tilt corrections
  float y_tilt_offset = (pen_length / 100) * sin(pitch_angle_rad);
  float x_tilt_offset = (pen_length / 100) * cos(roll_angle_rad);

  // curr_x -= x_tilt_offset;
  curr_y += y_tilt_offset;

  // Map to screen coordinates
  // Center the coordinates and scale
  int cursor_x = int((curr_x / screen_width) * res_x);
  int cursor_y = int((curr_y / screen_height) * res_y);

  // Optional: Add boundary checks
  cursor_x = max(0, min(cursor_x, res_x - 1));
  cursor_y = max(0, min(cursor_y, res_y - 1));

  Serial.print("Calculated X: ");
  Serial.println(cursor_x);
  Serial.print("Calculated Y: ");
  Serial.println(cursor_y);
}
// // localization function
// void localization_algo(float roll_angle, float pitch_angle, float range_uwb_1, float range_uwb_2, float pen_length, float screen_width, float screen_height, int res_x, int res_y) {
//   // END GOAL: UPDATE CURR_X and CURR_Y for screen emulation

//   // process the UWB ranging data
//   float x_coord, y_coord, x_tilt_offset, y_tilt_offset;
//   float pitch_angle_rad, roll_angle_rad;
//   float opp_side_trig;
//   float adj_side_trig;

//   x_coord = (pow(range_uwb_1, 2) + pow(screen_width, 2) - pow(range_uwb_2, 2)) / (2 * screen_width);
//   adj_side_trig = pow(range_uwb_1, 2) + pow(screen_width, 2) - pow(range_uwb_2, 2);
//   opp_side_trig = sqrt(abs(pow(2 * range_uwb_1 * screen_width, 2) - pow(adj_side_trig, 2)));
//   y_coord = opp_side_trig / (2 * screen_width);

//   // take care of tilting

//   // for now, pitch angle is forward/backward tilt
//   // roll angle is left/right tilt
//   pitch_angle_rad = pitch_angle * PI / 180;
//   roll_angle_rad = roll_angle * PI / 180;

//   y_tilt_offset = (pen_length / 100) * sin(pitch_angle_rad); // convert from cm to m
//   x_tilt_offset = (pen_length / 100) * cos(roll_angle_rad);

//   // get distance calculation
//   //    curr_x = x_coord - x_tilt_offset;
//   curr_x = x_coord; // still unsure about the x tilting offset, might add extra restriction.
//   curr_y = y_coord + y_tilt_offset; // plus or minus depends on orientation of pen button vs dwm
//   Serial.print("y_coord val: ");
//   Serial.println(y_coord);
//   Serial.print("y_tilt_offset val: ");
//   Serial.println(y_tilt_offset);
//   Serial.print("curr_ y val: ");
//   Serial.println(curr_y);

//   // convert to screen pixel coordinates
//   cursor_x = int(curr_x * (res_x / (screen_width)));
//   cursor_y = int(curr_y * (res_y / (screen_height)));
//   Serial.print("cursor x val: ");
//   Serial.println(cursor_x);
//   Serial.print("cursor y val: ");
//   Serial.println(cursor_y);
//   return;

// }

// mouse emulation function

void send_mouse_emulation() {

  int delta_cursor_x, delta_cursor_y;
  int scroll_amount = 5; // default, test via visual feedback

  unsigned long press_duration;

  // check if button has been depressed for the first time, if yes, keep track of when.
  delta_cursor_x = cursor_x - prev_x;
  delta_cursor_y = cursor_y - prev_y;

  Serial.print("delta cur x val: ");
  Serial.println(delta_cursor_x);
  Serial.print("delta cur y val: ");
  Serial.println(delta_cursor_y);

  if (digitalRead(PEN_BUTTON) == LOW) {
    if (is_pen_pressed == false) {
      pen_button_start_time = millis();
      is_pen_pressed = true;
    }
    press_duration = millis() - pen_button_start_time;

    // dead zone implementation
    // if (delta_cursor_x < 2 && delta_cursor_y < 2) {
    //     // to avoid small unnecessary fluctuational changes
    //     delta_cursor_x = 0;
    //     delta_cursor_y = 0;
    // }

    // differentiate between scrolling, clicking, and moving
    if (press_duration >= SCROLL_THRES) {
      // implement scrolling here
      // differentiate between scroll up and down
      if (delta_cursor_y > 0) {
        // scroll up
        bleMouse.move(0, 0, scroll_amount);
        delay(10);
      }
      else if (delta_cursor_y < 0) {
        // scroll down
        bleMouse.move(0, 0, -scroll_amount);
        delay(10);
      }

      // else, static hold, no need to scroll

    }
    else {
      // implement smooth movement of the cursor, until the pen button is high
    }
  }
  else if (digitalRead(PEN_BUTTON) == HIGH && is_pen_pressed) {
    press_duration = millis() - pen_button_start_time;

    if (press_duration < CLICK_THRES) {
      // implement clicking here
      bleMouse.click(MOUSE_LEFT);
      delay(10); // delay to allow the host device react upon the Bluetooth HID command
    }

    is_pen_pressed = false;
  }

  // always move the pen based on the delta cursor x and cursor y
  bleMouse.move(delta_cursor_x, delta_cursor_y);
//  delay(10);
}

void update_range_buf(float buffer[], int &index, float new_range){
    buffer[index] = new_range;
    index = (index+1) % NUM_SAMPLES;
}

float moving_avg(float buffer[], int size){
    float sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }

    return sum/size;
}

// handler functions
void ranging_handler()
{
  // seek the anchor addresses

  uint16_t device_addr;
  float new_range;
  float acc_imu_mag; 
  float prev_vel, new_vel; 
  float acc_uwb_mag; 
  float gravity_y, gravity_z; 

  // update time variables
  delta_t_uwb = (millis() - prev_time_uwb) / 1000.0;

  // get imu acceleration 
  gravity_y = -sin(curr_pitch_angle * PI / 180) * 9.81;
  gravity_z = cos(curr_pitch_angle * PI / 180) * cos(curr_roll_angle * PI / 180) * 9.81;
  acc_imu_mag = sqrt(pow((acc_y - gravity_y), 2) + pow((acc_z - gravity_z), 2)); // only care about y and z 2-d coord 

  // loop until both the anchor devices are found
  // curr_device = DW1000Ranging.getDistantDevice();
  device_addr = DW1000Ranging.getDistantDevice()->getShortAddress();
  new_range = DW1000Ranging.getDistantDevice()->getRange();

  if (device_addr == ANCHOR_ADDR_1) {
    // We know data is from anchor one

    // get data for localization

    if (set_flag_1 && set_flag_1_2) {
      prev_vel = (prev_range_uwb_1 - prev_prev_range_uwb_1) / delta_t_uwb; 
      new_vel = (new_range - prev_range_uwb_1) / delta_t_uwb; 

      acc_uwb_mag = abs((new_vel - prev_vel) / delta_t_uwb);

      if (acc_uwb_mag - acc_imu_mag > ACC_THRES) {
        Serial.println("Acceleration threshold exceeded, ignoring sample");
        return; 
      }
    }

    update_range_buf(range_uwb_1_buf, uwb_1_buf_idx, new_range);

    if (count_idx_1 < NUM_SAMPLES){
        count_idx_1++;
        // pre_scale_1 = moving_avg(range_uwb_1_buf, count_idx_1); 
        curr_range_uwb_1 = moving_avg(range_uwb_1_buf, count_idx_1);
    }
    else {
        // pre_scale_1 = moving_avg(range_uwb_1_buf, NUM_SAMPLES); 
        curr_range_uwb_1 = moving_avg(range_uwb_1_buf, NUM_SAMPLES);
    }

    if (set_flag_1 && !set_flag_1_2) {
      set_flag_1_2 = 1; 
    }

    if (!set_flag_1) {
    //   weight_factor_1 = known_dist_1 / pre_scale_1; 
      set_flag_1 = 1; 
    }

    // get a more accurate representation of range from the scaling! 
    // if (weight_factor_1 != 0) {
      // if (pre_scale_1 < 0) {
        // curr_range_uwb_1 = -weight_factor_1 * pre_scale_1; 
      // }
      // else {curr_range_uwb_1 = weight_factor_1 * pre_scale_1; }
  //  }

    prev_prev_range_uwb_1 = prev_range_uwb_1; 
    prev_range_uwb_1 = curr_range_uwb_1;

    // PRINT FOR TESTING
    Serial.print("Data from anchor one: ");
    Serial.print("\nCurrent Range: ");
    Serial.print(curr_range_uwb_1);
    Serial.print(" m");

  }

  else if (device_addr == ANCHOR_ADDR_2) {
    // We know data is from anchor two

    if (set_flag_2 && set_flag_2_2) {
      prev_vel = (prev_range_uwb_2 - prev_prev_range_uwb_2) / delta_t_uwb; 
      new_vel = (new_range - prev_range_uwb_2) / delta_t_uwb; 

      acc_uwb_mag = abs((new_vel - prev_vel) / delta_t_uwb);

      if (acc_uwb_mag - acc_imu_mag > ACC_THRES) {
        Serial.println("Acceleration threshold exceeded, ignoring sample");
        return; 
      }
    }

    update_range_buf(range_uwb_2_buf, uwb_2_buf_idx, new_range);

    if (count_idx_2 < NUM_SAMPLES){
        count_idx_2++;
        // pre_scale_1 = moving_avg(range_uwb_2_buf, count_idx_2); 
        curr_range_uwb_2 = moving_avg(range_uwb_2_buf, count_idx_2);
    }
    else {
        // pre_scale_2 = moving_avg(range_uwb_2_buf, NUM_SAMPLES); 
        curr_range_uwb_2 = moving_avg(range_uwb_2_buf, NUM_SAMPLES);
    }

    if (set_flag_2 && !set_flag_2_2) {
      set_flag_2_2 = 1; 
    }

    if (!set_flag_2) {
    //   weight_factor_2 = known_dist_2 / pre_scale_2; 
      set_flag_2 = 1; 
    }

    // get a more accurate representation of range from the scaling! 
    // if (weight_factor_2 != 0) {
      // if (pre_scale_2 < 0) {
        // curr_range_uwb_2 = -weight_factor_2 * pre_scale_2; 
      // }
      // else {curr_range_uwb_2 = weight_factor_2 * pre_scale_2; }
  //  }

    prev_prev_range_uwb_2 = prev_range_uwb_2; 
    prev_range_uwb_2 = curr_range_uwb_2;

    // PRINT FOR TESTING
    Serial.print("Data from anchor two: ");
    Serial.print("\nCurrent Range: ");
    Serial.print(curr_range_uwb_2);
    Serial.print(" m");

  }

  ranging_flag = 1;
  prev_time_uwb = millis(); 
}

void new_dev_handler(DW1000Device* dev)
{
  // connection with uwb anchors
  Serial.print("a new device is connected, ready for ranging!");
  Serial.print("\tdevice short address: \n");
  Serial.print(dev->getShortAddress(), HEX);
}

void inactive_handler(DW1000Device* dev)
{
  Serial.print("Inactivity detected!");
  Serial.print("\tInactive device short address: \n");
  Serial.print(dev->getShortAddress(), HEX);
}