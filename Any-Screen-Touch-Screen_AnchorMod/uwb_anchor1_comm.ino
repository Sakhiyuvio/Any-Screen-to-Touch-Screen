
// include necessary libraries 

#include <SPI.h> 
#include "DW1000Ranging.h"

#define ANCHOR_ADDR_1 "86:17:5B:D5:A9:9A:E2:9C" // use reference from makerlab for now
#define PEN_UWB_ADDR "86:17:5B:D5:A9:9A:E2:9A" // pen

// ESP32-S3 SPI pin config
#define SPI_SCLK 20 
#define SPI_MISO 21 
#define SPI_MOSI 19
#define SPI_CS 18 

// data rate 
#define comm_data_rate 115920 // communication of dwm1000 > 110kbps, stretch a bit higher for delay and accuracy purposes
#define comm_init_delay 1000 // 1 s 

// DWM1000 pin config
// ranging library utilizes uint8_t for parameters
const uint8_t RST_pin = 5;
const uint8_t CS_pin = 18;
const uint8_t INT_pin = 39;

void setup()
{
    // start communication protocol initialization
    Serial.begin(comm_data_rate);
    delay(comm_init_delay);

    // SPI and DWM1000 initialization
    SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, SPI_CS);
    DW1000Ranging.initCommunication(RST_pin, CS_pin, INT_pin);

    // create and call handlers here, to get ranging data, LED, and device activation
    DW1000Ranging.attachNewRange(ranging_handler); // process distance data between anchor and pen
    DW1000Ranging.attachBlinkDevice(new_dev_handler); 
    DW1000Ranging.attachInactiveDevice(inactive_handler);

    // dsp pipeline here, filtering dist data and/or improve accuracy

    // start dwm as an anchor module, first anchor
    DW1000Ranging.startAsAnchor(ANCHOR_ADDR_1, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);    
}

// continuous looping for uwb real-time data processing 
void loop()
{
    // continuously loop to gather ranging data, 
    // this function handles the Two-Way Ranging Algorithm to accurately measure distance 
    // between anchor and pen. 
    DW1000Ranging.loop()
}

// handler functions
void ranging_handler()
{
    // get info of tag addr, only perform ranging if talking to pen uwb 
    if(DW1000Ranging.getDistantDevice()->getShortAddress() == PEN_UWB_ADDR){ // define pen_addr
        Serial.print("Data from: ");
        Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX); // print hex address 
        Serial.print("\nCurrent Range: ");
        Serial.print(DW1000Ranging.getDistantDevice()->getRange()); 
        Serial.print(" m"); // distance measurement unit
    }
    else {
        Serial.print("Data Invalid (Unknown)");
    }
    return; 
    // might need to analyze signal power here, skip for now 
}

// LED blinks to indicate device connection
void new_dev_handler(DW1000Device *dev)
{
    // connection with a new uwb sensor device 
    Serial.print("blink, a new device is connected!");
    Serial.print("\tdevice short address: \n");
    Serial.print(dev->getShortAddress(), HEX);
}

void inactive_handler(DW1000Device *dev)
{

    // debugging purpose, prints inactivity 
    Serial.print("Inactivity detected!");
    Serial.print("\tinactive device short address: \n");
    Serial.print(dev->getShortAddress(), HEX);
}

