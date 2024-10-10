
#include <SPI.h>
#include "DW1000Ranging.h"

#define PEN_UWB_ADDR "86:17:5B:D5:A9:9A:E2:9A" // unique addr

// anchor short addresses
#define ANCHOR_ADDR_1 0xE29C
#define ANCHOR_ADDR_2 0xE29D 
// ESP32-S3 SPI pin config
#define SPI_SCLK 20
#define SPI_MISO 21
#define SPI_MOSI 19
#define SPI_CS 18

// data rate for pen comm
#define comm_data_rate 115920
#define comm_init_delay 1000 

const uint8_t RST_pin = 5;
const uint8_t CS_pin = 18;
const uint8_t INT_pin = 39; 

void setup()
{
    Serial.begin(coomm_data_rate);
        
    // set up bluetooth/WiFi config here for communicating data

    // add logic for Bluetooth HID as well?

    // init uwb communication
    SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, SPI_CS);
    DW1000Ranging.initCommunication(RST_pin, CS_pin, INT_pin);

    // handlers of getting data and connecting to other uwb transceivers
    DW1000Ranging.attachNewRange(ranging_handler);
    DW1000Ranging.attachNewDevice(new_dev_handler);
    DW1000Ranging.attachInactiveDevice(inactive_handler);

    // start DWM as tag 
    DW1000Ranging.startAsTag(PEN_UWB_ADDR, DW1000.MODE_LONGDATA_RANGE_LOWPOWER);

    // set up data post-processing to host device here, via Bluetooth / WiFi

}

void loop() // Arduino IDE, continuous looping 
{
    DW1000Ranging.loop(); // tag-anchor communication
    
    // add logic, set a timeout based protocol to send data 
    // and signal to host device, potentially through input pin from the pen as well
    // to initiate Bluetooth HID

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

    // loop until the anchor devices are found 
    while (!(anchor_one_flag) && !(anchor_two_flag)){
        device_addr = DW1000Ranging.getDistantDevice()->getShortAddress();

        if(device_addr == ANCHOR_ADDR_1 && !anchor_one_flag) {
           // We know data is from anchor one
           Serial.print("Data from anchor one: ");
           Serial.print("\nCurrent Range: ");
           Serial.print(device_addr->getRange());
           Serial.print(" m");
           // set flag
           anchor_one_flag = 1;        
        }

        else if (device_addr == ANCHOR_ADDR_2 && !anchor_two_flag) {
           // We know data is from anchor one
           Serial.print("Data from anchor one: ");
           Serial.print("\nCurrent Range: ");
           Serial.print(device_addr->getRange());
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


