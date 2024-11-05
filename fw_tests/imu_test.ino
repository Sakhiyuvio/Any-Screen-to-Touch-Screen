#include <SPI.h>
#include "LSM6DSLSensor.h"

// ESP32-S3 SPI pin config - IMU
#define IMU_SPI_SCLK 18
#define IMU_SPI_MISO 23
#define IMU_SPI_MOSI 19
#define IMU_SPI_CS 5

// serial comm data rate
#define comm_data_rate 115920
#define comm_init_delay 1000 

// pi
#define PI 3.14159265358979323846

// gravitational constant
#define g 9.8

SPIClass imu_dev_spi(IMU_SPI_MOSI, IMU_SPI_MISO, IMU_SPI_SCLK);
LSM6DSLSensor acc_gyr(&imu_dev_spi, IMU_SPI_CS); 

// global variables for imu
int32_t accelerometer_sensor[3]; 
int32_t gyroscope_sensor[3];
int32_t acc_x, acc_y, acc_z; 
int32_t gyr_x, gyr_y; 
float acc_roll_angle, acc_pitch_angle;
float gyr_roll_angle, gyr_pitch_angle; 
float curr_pitch_angle;
float curr_roll_angle; 
float trust_factor = 0.95; 
float trust_factor_complement = 1 - trust_factor; 

// time-keeping
float delta_t;
unsigned long prev_time; 

void setup()
{
    Serial.begin(comm_data_rate);

    // init IMU communication
    imu_dev_spi.begin();
    if (acc_gyr.begin() == 0) {
        Serial.print("IMU initialized successfully");
    }
    else {
        Serial.print("IMU initialized failed");
    }

    // consider delaying after init on sensor 
    if (acc_gyr.Enable_X() == 0) {
        Serial.print("IMU accelerometer connected successfully");
    }
    else {
        Serial.print("IMU accelerometer connection failed");
    }
    if (acc_gyr.Enable_G() == 0) {
        Serial.print("IMU gyroscope connected successfully");
    }
    else {
        Serial.print("IMU gyroscope connection failed");
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
}

void loop()
{
    unsigned long print_prev_time; 
    print_prev_time = millis(); 
    // set up ranging loop for the IMU here
    acc_gyr.Get_X_Axes(accelerometer_sensor);
    acc_gyr.Get_G_Axes(gyroscope_sensor);

    acc_x = accelerometer_sensor[0];
    acc_y = accelerometer_sensor[1];
    acc_z = accelerometer_sensor[2]; 
    gyr_x = gyroscope_sensor[0];
    gyr_y = gyroscope_sensor[1]; 

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
        Serial.print("Pitch: ");
        Serial.print(curr_pitch_angle);
        Serial.print(" degrees, Roll: ");
        Serial.print(curr_roll_angle);
        Serial.print(" degrees");
        print_prev_time = millis(); 
    }

}
