
#include <SPI.h>
#include "DW1000Ranging.h"

#define PEN_UWB_ADDR "86:17:5B:D5:A9:9A:E2:9A" // unique addr

// anchor addresses
#define ANCHOR_ADDR_1 "86:17:5B:D5:A9:9A:E2:9C"
#define ANCHOR_ADDR_2 "86:17:5B:D5:A9:9A:E2:9D"
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
    uint16_t device_addr = 
}


