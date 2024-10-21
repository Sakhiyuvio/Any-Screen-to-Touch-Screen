
#include <SPI.h>
#include "DW1000Ranging.h"
#include "LSM6DSLSensor.h"
#include <BleMouse.h>
#include <BLEDevice.h>

#define PEN_UWB_ADDR "86:17:5B:D5:A9:9A:E2:9A" // unique addr

// anchor short addresses
#define ANCHOR_ADDR_1 0xE29C
#define ANCHOR_ADDR_2 0xE29D 

// ESP32-S3 SPI pin config - UWB
#define SPI_SCLK 20
#define SPI_MISO 21
#define SPI_MOSI 19
#define SPI_CS 18

// ESP32-S3 SPI pin config - IMU
#define IMU_SPI_SCLK 12
#define IMU_SPI_MISO 17
#define IMU_SPI_MOSI 7
#define IMU_SPI_CS 6

// ESP32-S3 pen-button pin
#define PEN_BUTTON 23 

// data rate for pen comm
#define comm_data_rate 115920
#define comm_init_delay 1000 

const uint8_t RST_pin = 5;
const uint8_t CS_pin = 18;
const uint8_t INT_pin = 39; 

// global SPI class and lsm6dsl instance for the IMU
SPIClass imu_dev_spi(IMU_SPI_MOSI, IMU_SPI_MISO, IMU_SPI_SCLK);
LSM6DSLSensor acc_gyr(&imu_dev_spi, IMU_SPI_CS);    

// global variables for ranging storage - uwb
float curr_range_uwb_1;
float curr_range_uwb_2; 

// global variables for ranging storage - imu
int32_t accelerometer_sensor[3]; 
int32_t gyroscope_sensor[3];

// global variables to keep track of current screen coordinates
float curr_x, curr_y; 

// bluetooth mouse instance
BLEMouse mouse_em;

void setup()
{
    Serial.begin(coomm_data_rate);

    // init IMU communication
    imu_dev_spi.begin();
    acc_gyr.begin();
    acc_gyr.Enable_X();
    acc_gyr.Enable_G();

    // init pen button
    pinMode(PEN_BUTTON, INPUT);
    // enabling internal pullup
    digitalWrite(PEN_BUTTON, HIGH);

    // init uwb communication
    SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, SPI_CS);
    DW1000Ranging.initCommunication(RST_pin, CS_pin, INT_pin);
    curr_range_uwb_1 = 0; 
    curr_range_uwb_2 = 0; 
    // handlers of getting data and connecting to other uwb transceivers
    DW1000Ranging.attachNewRange(ranging_handler);
    DW1000Ranging.attachNewDevice(new_dev_handler);
    DW1000Ranging.attachInactiveDevice(inactive_handler);
    // start DWM as tag 
    DW1000Ranging.startAsTag(PEN_UWB_ADDR, DW1000.MODE_LONGDATA_RANGE_LOWPOWER);

    // TO DO: set up data post-processing to host device here, via Bluetooth BLE

    // bluetooth HID init
    BLEDevice::init("ESP32-MOUSE-HID");
    mouse_em.begin();

}

void loop() // Arduino IDE, continuous looping 
{
    DW1000Ranging.loop(); // tag-anchor communication

    // set up ranging loop for the IMU here
    acc_gyr.Get_X_Axes(accelerometer_sensor);
    acc_gyr.Get_G_Axes(gyroscope_sensor);

    // at each time instance, we need to be prepared to send coordinates of the pen on the screen
    localization_algo(accelerometer_sensor, gyroscope_sensor); 

    // process these data to replicate bluetooth HID
    if (digitalread(PEN_BUTTON) == LOW) {
        // Bluetooth HID emulation here, use the coordinates received after localization
        if (mouse_em.isConnected()) {
            send_mouse_emulation(); 
            // CONSIDER DELAYS
        }
    }

    // add logic, set a timeout based protocol to send data 
    // and signal to host device for monitoring


}

// localization function
void localization_algo(int32_t* accelerometer, int32_t* gyroscope_sensor) {
    // END GOAL: UPDATE CURR_X and CURR_Y for screen emulation 
}

// mouse emulation function
void send_mouse_emulation() {
    
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
    Instead of only printing, store and send data to host device for HID 
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


