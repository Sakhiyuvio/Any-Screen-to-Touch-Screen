#include <SPI.h>

#define IMU_SPI_SCLK 8
#define IMU_SPI_MISO 9
#define IMU_SPI_MOSI 7
#define IMU_SPI_CS 6

SPIClass imu_spi;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("Starting basic SPI test...");
    
    // Configure SPI
    pinMode(IMU_SPI_CS, OUTPUT);
    digitalWrite(IMU_SPI_CS, HIGH);
    
    imu_spi.begin(IMU_SPI_SCLK, IMU_SPI_MISO, IMU_SPI_MOSI, IMU_SPI_CS);
    
    // Test CS pin
    Serial.println("Testing CS pin control...");
    digitalWrite(IMU_SPI_CS, LOW);
    delayMicroseconds(100);
    Serial.println("CS is LOW");
    digitalWrite(IMU_SPI_CS, HIGH);
    delayMicroseconds(100);
    Serial.println("CS is HIGH");
    
    // Explicit SPI mode setting
    imu_spi.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE3)); // Slower speed for testing
    
    // WHO_AM_I register read with detailed steps
    Serial.println("Attempting WHO_AM_I register read...");
    
    digitalWrite(IMU_SPI_CS, LOW);
    delayMicroseconds(100);
    
    // Send read command
    uint8_t cmd = 0x0F | 0x80;  // WHO_AM_I register (0x0F) with read bit set (0x80)
    uint8_t sent = imu_spi.transfer(cmd);
    Serial.printf("Sent command: 0x%02X\n", cmd);
    
    delayMicroseconds(100);
    
    // Read response
    uint8_t whoami = imu_spi.transfer(0x00);
    Serial.printf("Received value: 0x%02X\n", whoami);
    
    digitalWrite(IMU_SPI_CS, HIGH);
    imu_spi.endTransaction();
    
    // Try multiple reads
    Serial.println("\nTrying multiple reads with delays...");
    for(int i = 0; i < 5; i++) {
        delay(100);  // Longer delay between attempts
        
        digitalWrite(IMU_SPI_CS, LOW);
        delayMicroseconds(100);
        
        imu_spi.transfer(0x0F | 0x80);
        delayMicroseconds(100);
        whoami = imu_spi.transfer(0x00);
        
        digitalWrite(IMU_SPI_CS, HIGH);
        
        Serial.printf("Read attempt %d: 0x%02X\n", i+1, whoami);
    }
}

void loop() {
    delay(1000);
}



// #include <Adafruit_LSM6DSL.h>

// // ESP32-S3 SPI pin config - IMU
// #define IMU_SPI_SCLK 8
// #define IMU_SPI_MISO 9
// #define IMU_SPI_MOSI 7
// #define IMU_SPI_CS 6
// //#define IMU_INT_1 18
// //#define IMU_INT_2 19

// // serial comm data rate
// #define comm_data_rate 115200
// #define comm_init_delay 1000

// // pi
// #define PI 3.14159265358979323846

// // gravitational constant
// #define g 9.8

// SPIClass imu_spi;
// Adafruit_LSM6DSL imu_dsl;

// // global variables for imu
// int32_t acc_x, acc_y, acc_z;
// int32_t gyr_x, gyr_y;
// float acc_roll_angle, acc_pitch_angle;
// float gyr_roll_angle, gyr_pitch_angle;  
// float curr_pitch_angle;
// float curr_roll_angle;
// float trust_factor = 0.95;
// float trust_factor_complement = 1 - trust_factor;

// // time-keeping
// float delta_t;
// unsigned long prev_time;

// void setup()
// {
//     Serial.begin(comm_data_rate);
//     while (!Serial) {
//       delay(10);
//     }

//     Serial.println("Testing IMU!");
//     // Configure pins
//     pinMode(IMU_SPI_CS, OUTPUT);
//     digitalWrite(IMU_SPI_CS, HIGH);  // Initially HIGH
//     imu_spi.begin(IMU_SPI_SCLK, IMU_SPI_MISO, IMU_SPI_MOSI, IMU_SPI_CS);
//     imu_spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3)); // Note: MODE3
   
//     delay(10);
//     // Try to read WHO_AM_I register directly
//     Serial.println("Testing SPI communication...");
//     uint8_t whoami = readRegister(0x0F);  // WHO_AM_I register
//     Serial.printf("WHO_AM_I register value: 0x%02X (Should be 0x6A)\n", whoami);

//     Serial.println("Attempting to initialize LSM6DSL...");
//     if(!imu_dsl.begin_SPI(IMU_SPI_CS, &imu_spi)){
//       if(!imu_dsl.begin_SPI(IMU_SPI_CS, IMU_SPI_SCLK, IMU_SPI_MISO, IMU_SPI_MOSI)){
//          Serial.println("Failed to find LSM6DSL chip");
//        }  
//     }
//     else {
//       Serial.println("IMU initialized successfully");
//     }
 
// //    // init IMU communication
// //    pinMode(IMU_INT_1, INPUT_PULLDOWN);
// //    pinMode(IMU_INT_2, INPUT_PULLDOWN);
// //  
//     acc_x = 0;
//     acc_y = 0;
//     acc_z = 0;
//     gyr_x = 0;
//     gyr_y = 0;
//     curr_pitch_angle = 0.0;
//     curr_roll_angle = 0.0;
//     acc_roll_angle = 0.0;
//     acc_pitch_angle = 0.0;
//     gyr_roll_angle = 0.0;
//     gyr_pitch_angle = 0.0;
//     delta_t = 0.0;
//     prev_time = millis();
// }

// void loop()
// {
//     unsigned long print_prev_time;
//     print_prev_time = millis();
//     // set up ranging loop for the IMU here
//     sensors_event_t accel;
//     sensors_event_t gyro;
//     sensors_event_t temp;
//     imu_dsl.getEvent(&accel, &gyro, &temp);

//     acc_x = accel.acceleration.x;
//     acc_y = accel.acceleration.y;
//     acc_z = accel.acceleration.z;
//     gyr_x = gyro.gyro.x;
//     gyr_y = gyro.gyro.y;
//     // perform adaptive noise filtering on the IMU sensor (Optional for now)

//     // update time variables
//     delta_t = (millis() - prev_time) / 1000.0;
//     prev_time = millis();

//     // process the roll and pitch angle for configurating tilting through IMU sensor
//     acc_roll_angle = atan2(acc_y/g, acc_z/g) * 180/PI;
//     acc_pitch_angle = atan2(acc_x/g, acc_z/g) * 180/PI;
//     gyr_roll_angle = gyr_x * delta_t;
//     gyr_pitch_angle = gyr_y * delta_t;

//     // complementary filter imp
//     curr_roll_angle = (curr_roll_angle + gyr_roll_angle) * trust_factor + acc_roll_angle*trust_factor_complement;
//     curr_pitch_angle = (curr_pitch_angle - gyr_pitch_angle) * trust_factor + acc_pitch_angle*trust_factor_complement;

//     if (millis() - print_prev_time > 1000) {
//         Serial.print("Pitch: ");
//         Serial.print(curr_pitch_angle);
//         Serial.print(" degrees, Roll: ");
//         Serial.print(curr_roll_angle);
//         Serial.print(" degrees");
//         print_prev_time = millis();
//     }

// }

// uint8_t readRegister(uint8_t reg) {
//     digitalWrite(IMU_SPI_CS, LOW);
//     delayMicroseconds(10);
//     imu_spi.transfer(reg | 0x80);  // Set read bit
//     uint8_t value = imu_spi.transfer(0x00);
//     digitalWrite(IMU_SPI_CS, HIGH);
//     return value;
// }
