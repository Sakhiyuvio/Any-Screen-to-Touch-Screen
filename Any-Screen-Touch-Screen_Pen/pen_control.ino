
#include <SPI.h>
#include "DW1000Ranging.h"
#include <BleMouse.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Adafruit_LSM6DS.h>
#include <Adafruit_LSM6DSL.h>

#define PEN_UWB_ADDR "86:17:5B:D5:A9:9A:E2:9A" // unique addr
#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1"

// anchor short addresses
#define ANCHOR_ADDR_1 0xE29C
#define ANCHOR_ADDR_2 0xE29D 

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
#define SCROLL_THRES 1000
#define CLICK_THRES 100

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
float curr_range_uwb_1;
float curr_range_uwb_2; 

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
float delta_t;
unsigned long prev_time; 

// global variables to keep track of current screen coordinates
float curr_x, curr_y; 
float prev_x, prev_y; 

// bluetooth mouse instance
BLEMouse bleMouse;

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
    if(!imu_dsl.begin_SPI(IMU_SPI_CS, &SPI2)){
      if(!imu_dsl.begin_SPI(IMU_SPI_CS, IMU_SPI_SCLK, IMU_SPI_MISO, IMU_SPI_MOSI)){
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
    pinMode(PEN_BUTTON, INPUT);
    // enabling internal pullup
    digitalWrite(PEN_BUTTON, HIGH);

    // init uwb communication
    SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, SPI_CS);
    DW1000Ranging.initCommunication(RST_pin, CS_pin, INT_pin);
    // consider delaying 
    curr_range_uwb_1 = 0; 
    curr_range_uwb_2 = 0; 
    // handlers of getting data and connecting to other uwb transceivers
    DW1000Ranging.attachNewRange(ranging_handler);
    DW1000Ranging.attachNewDevice(new_dev_handler);
    DW1000Ranging.attachInactiveDevice(inactive_handler);
    // start DWM as tag 
    DW1000Ranging.startAsTag(PEN_UWB_ADDR, DW1000.MODE_LONGDATA_RANGE_LOWPOWER);

    // TO DO: set up data post-processing to host device here, via Bluetooth BLE
    // TO DO: I think the cursor should be initialized during calibration process 
    // set to 0 for now
    prev_x = 0; 
    prev_y = 0;
    curr_x = 0; 
    curr_y = 0; 


    // BLE dev and HID init
    BLEDevice::init("ESP32-BLE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    // BLE service
    BLEService *pService = pServer->createService(SERVICE_UUID); // service uuid
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID, // char uuid
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );

    pService->start();
    pServer->getAdvertising()->start();
    Serial.println("BLE Waiting for GUI Connection...");
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
    prev_time = millis();

    // process the roll and pitch angle for configurating tilting through IMU sensor
    acc_roll_angle = atan2(acc_y/g, acc_z/g) * 180/PI;
    acc_pitch_angle = atan2(acc_x/g, acc_z/g) * 180/PI;
    gyr_roll_angle = gyr_x * delta_t; 
    gyr_pitch_angle = gyr_y * delta_t; 

    // complementary filter imp
    curr_roll_angle = (curr_roll_angle + gyr_roll_angle) * trust_factor + acc_roll_angle*trust_factor_complement;
    curr_pitch_angle = (curr_pitch_angle - gyr_pitch_angle) * trust_factor + acc_pitch_angle*trust_factor_complement;

    // at each time instance, we need to be prepared to send coordinates of the pen on the screen
    // pass in the tilting angle, as well as ranging parameters
    localization_algo(curr_roll_angle, curr_pitch_angle, curr_range_uwb_1, curr_range_uwb_2); 

    // add logic, set a timeout based protocol to send data 
    // and signal to host device for monitoring
    if (host_dev_connected) {
        // send binary buffer
        uint8_t data[8]; // per byte
        memcpy(data, &curr_x, sizeof(float));
        memcpy(data + sizeof(float), &curr_y, sizeof(float));
        pCharacteristic->setValue(data, 8);
        pCharacteristic->notify();
    }

    // process these data to replicate bluetooth HID        // Bluetooth HID emulation here, use the coordinates received after localization
    if (bleMouse.isConnected()) {
        send_mouse_emulation(); 
        // CONSIDER DELAYS, use visual feedback to see if there are lags due to host device being overwhelmed 
    }
}

// localization function
void localization_algo(float roll_angle, float pitch_angle, float range_uwb_1, float range_uwb_2, float screen_width, float pen_length) {
    // END GOAL: UPDATE CURR_X and CURR_Y for screen emulation 

    // process the UWB ranging data 
    float x_coord, y_coord, y_tilt_offset;
    float pitch_angle_rad;
    float opp_side_trig; 
    float adj_side_trig 
    
    x_coord = (pow(range_uwb_1, 2) + pow(screen_width, 2) - pow(range_uwb_2, 2)) / 2;
    adj_side_trig = pow(range_uwb_1, 2) + pow(screen_width, 2) - pow(range_uwb_2, 2);
    opp_side_trig = sqrt(pow(2*range_uwb_1, 2) - pow(adj_side_trig, 2));
    y_coord = opp_side_trig / 2; 

    // take care of tilting

    // for now, pitch angle is forward/backward tilt
    // roll angle is left/right tilt 
    pitch_angle_rad = pitch_angle * PI / 180; 
    roll_angle_rad = roll_angle * PI / 180;

    y_tilt_offset = pen_length * cos(pitch_angle_rad);
    x_tilt_offset = pen_length * cos(roll_angle_rad);

    curr_x = x_coord - x_tilt_offset;
    curr_y = y_coord - y_tilt_offset;

    return; 
}

// mouse emulation function

/* TO DO: CHECK IF CURSOR X AND Y GOES BEYOND LIMIT */ 
void send_mouse_emulation() {

    int delta_cursor_x, int delta_cursor_y; 
    int scroll_amount = 10; // default, test via visual feedback 

    unsigned long press_duration; 

    // check if button has been depressed for the first time, if yes, keep track of when. 
    
    if (digitalRead(PEN_BUTTON) == LOW) { 
        if(is_pen_pressed == false) {
            pen_button_start_time = millis();
            is_pen_pressed = true;
        }
        press_duration = millis() - pen_button_start_time; 
        delta_cursor_x = curr_x - prev_x;
        delta_cursor_y = curr_y - prev_y; 

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
            bleMouse.move(delta_cursor_x, delta_cursor_y); 
            delay(10);
        }
    }
    else if (digitalRead(PEN_BUTTON) == HIGH && is_pen_pressed) {
        press_duration = millis() - pen_button_start_time;

        if (press_duration < CLICK_THRES){
            // implement clicking here 
            bleMouse.click(MOUSE_LEFT);
            delay(10); // delay to allow the host device react upon the Bluetooth HID command 
        }

        is_pen_pressed = false; 
    }

}

// handler functions
void ranging_handler()
{
    // seek the anchor addresses
    // while not found
    int anchor_one_flag = 0;
    int anchor_two_flag = 0; 
    uint16_t device_addr; 

    /* 
    TO DO: 
    Instead of only printing, store and send data to host device for HID (MUTHU)
    */

    // loop until both the anchor devices are found 
    while (!(anchor_one_flag) && !(anchor_two_flag)){
        curr_device = DW1000Ranging.getDistantDevice();
        device_addr = DW1000Ranging.getDistantDevice()->getShortAddress();

        if(device_addr == ANCHOR_ADDR_1 && !anchor_one_flag) {
           // We know data is from anchor one

           // send data for localization
           curr_range_uwb_1 = curr_device->getRange();  

           // PRINT FOR TESTING
           Serial.print("Data from anchor one: ");
           Serial.print("\nCurrent Range: ");
           Serial.print(curr_range_uwb_1);
           Serial.print(" m");
        
           // set flag
           anchor_one_flag = 1;    
   
        }

        else if (device_addr == ANCHOR_ADDR_2 && !anchor_two_flag) {
           // We know data is from anchor two

           // send data for localization
           curr_range_uwb_2 = curr_device->getRange();  

           // PRINT FOR TESTING
           Serial.print("Data from anchor one: ");
           Serial.print("\nCurrent Range: ");
           Serial.print(curr_range_uwb_2);
           Serial.print(" m");
           // set flag
           anchor_two_flag = 1;    
        }
    }
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

