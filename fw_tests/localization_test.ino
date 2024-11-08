#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LSM6DS.h>
#include <Adafruit_LSM6DSL.h>

#include <SPI.h>

// ESP32-S3 SPI pin config - IMU
#define IMU_SPI_SCLK 8
#define IMU_SPI_MISO 9
#define IMU_SPI_MOSI 7
#define IMU_SPI_CS 6
//#define IMU_INT_1 18
//#define IMU_INT_2 19

// serial comm data rate
#define comm_data_rate 115200
#define comm_init_delay 1000

// gravitational constant
#define g 9.8

SPIClass SPI2;
Adafruit_LSM6DSL imu_dsl;

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
unsigned long print_prev_time;

// global variables to keep track of current screen coordinates
float curr_x, curr_y;
int cursor_x, cursor_y;

void setup()
{
    Serial.begin(comm_data_rate);
    while (!Serial) {
      delay(10);
    }

    Serial.println("Testing IMU!");
    // Configure pins
    pinMode(IMU_SPI_CS, OUTPUT);
    digitalWrite(IMU_SPI_CS, HIGH);  // Initially HIGH
    SPI2.begin(IMU_SPI_SCLK, IMU_SPI_MISO, IMU_SPI_MOSI, IMU_SPI_CS);
   
    delay(10);
    // Try to read WHO_AM_I register directly
    Serial.println("Testing SPI communication...");
    uint8_t whoami = readRegister(0x0F);  // WHO_AM_I register
    Serial.printf("WHO_AM_I register value: 0x%02X (Should be 0x6A)\n", whoami);

    Serial.println("Attempting to initialize LSM6DSL...");
    if(!imu_dsl.begin_SPI(IMU_SPI_CS, &SPI2)){
      if(!imu_dsl.begin_SPI(IMU_SPI_CS, IMU_SPI_SCLK, IMU_SPI_MISO, IMU_SPI_MOSI)){
         Serial.println("Failed to find LSM6DSL chip");
       }  
    }
    else {
      Serial.println("IMU initialized successfully");
    }
 
//    // init IMU communication
//    pinMode(IMU_INT_1, INPUT_PULLDOWN);
//    pinMode(IMU_INT_2, INPUT_PULLDOWN);
//  
    curr_x = 0.0;
    curr_y = 0.0;
    cursor_x = 0;
    cursor_y = 0;
   
    acc_x = 0.0;
    acc_y = 0.0;
    acc_z = 0.0;
    gyr_x = 0.0;
    gyr_y = 0.0;
    curr_pitch_angle = 0.0;
    curr_roll_angle = 0.0;
    acc_roll_angle = 0.0;
    acc_pitch_angle = 0.0;
    gyr_roll_angle = 0.0;
    gyr_pitch_angle = 0.0;
    delta_t = 0.0;
    prev_time = millis();
    print_prev_time = millis();

}

void loop()
{
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

    if (millis() - print_prev_time > 1000) {
        Serial.print(curr_pitch_angle);
        Serial.print(", ");
        Serial.println(curr_roll_angle);
        print_prev_time = millis();
    }

    localization_algo_test(curr_roll_angle, curr_pitch_angle);

}

// localization function
void localization_algo(float roll_angle, float pitch_angle, float range_uwb_1, float range_uwb_2, float pen_length, float screen_width, float screen_height, int res_x, int res_y) {
    // END GOAL: UPDATE CURR_X and CURR_Y for screen emulation

    // process the UWB ranging data
    float x_coord, y_coord, y_tilt_offset;
    float pitch_angle_rad;
    float opp_side_trig;
    float adj_side_trig
   
    x_coord = (pow(range_uwb_1, 2) + pow(screen_width, 2) - pow(range_uwb_2, 2)) / 2*screen_width;
    adj_side_trig = pow(range_uwb_1, 2) + pow(screen_width, 2) - pow(range_uwb_2, 2);
    opp_side_trig = sqrt(pow(2*range_uwb_1, 2) - pow(adj_side_trig, 2));
    y_coord = opp_side_trig / 2*screen_width;

    // take care of tilting

    // for now, pitch angle is forward/backward tilt
    // roll angle is left/right tilt
    pitch_angle_rad = pitch_angle * PI / 180;
    roll_angle_rad = roll_angle * PI / 180;

    y_tilt_offset = pen_length / 100 * sin(pitch_angle_rad); // convert from cm to m
    x_tilt_offset = pen_length / 100 * cos(roll_angle_rad);

    // get distance calculation
//    curr_x = x_coord - x_tilt_offset;
    curr_x = x_coord; // still unsure about the x tilting offset, might add extra restriction.
    curr_y = y_coord - y_tilt_offset;

    // convert to screen pixel coordinates
    cursor_x = int(curr_x * (res_x / screen_width));
    cursor_y = int(curr_y * (res_y / screen_height));
    return;
}

// localization test cases
void localization_algo_test(float roll_angle, float pitch_angle) {
  // test localization algo: no pitch and roll
  localization_algo(0, 0, 0.2, 0.2, 1, 0.5, 0.3, 255, 255);
   
}


// SPI debugging
uint8_t readRegister(uint8_t reg) {
    digitalWrite(IMU_SPI_CS, LOW);
    delayMicroseconds(10);
    SPI2.transfer(reg | 0x80);  // Set read bit
    uint8_t value = SPI2.transfer(0x00);
    digitalWrite(IMU_SPI_CS, HIGH);
    return value;
}