
// include necessary libraries 

#include <SPI.h> 
#include "DW1000Ranging.h"

#define PEN_UWB_ADDR "7D:00:22:EA:82:60:3B:9C" // unique addr

// ESP32-S3 SPI pin config
#define SPI_SCLK 12
#define SPI_MISO 13
#define SPI_MOSI 11
#define SPI_CS 10

// data rate 
#define comm_data_rate 115200 // communication of dwm1000 > 110kbps, stretch a bit higher for delay and accuracy purposes
#define comm_init_delay 1000 // 1 s 

// DWM1000 pin config
// ranging library utilizes uint8_t for parameters
const uint8_t RST_pin = 7;
const uint8_t CS_pin = 10;
const uint8_t INT_pin = 4;

void setup()
{
    // start communication protocol initialization
    Serial.begin(comm_data_rate);
    delay(comm_init_delay);

    // SPI and DWM1000 initialization
    SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(RST_pin, CS_pin, INT_pin);

    // create and call handlers here, to get ranging data, LED, and device activation
    DW1000Ranging.attachNewRange(newRange); // process distance data between anchor and pen
    DW1000Ranging.attachNewDevice(newDevice); 
    DW1000Ranging.attachInactiveDevice(inactiveDevice);

    // dsp pipeline here, filtering dist data and/or improve accuracy

    // start dwm as an anchor module, first anchor
    DW1000Ranging.startAsTag(PEN_UWB_ADDR, DW1000.MODE_LONGDATA_RANGE_LOWPOWER);    
}

// continuous looping for uwb real-time data processing 
void loop()
{
    // continuously loop to gather ranging data, 
    // this function handles the Two-Way Ranging Algorithm to accurately measure distance 
    // between anchor and pen. 
    DW1000Ranging.loop();
}

// handler functions
void newRange()
{
    // get info of tag addr, only perform ranging if talking to pen uwb 
    Serial.print("Data from: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX); // print hex address 
    Serial.print("\nCurrent Range: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRange()); 
    Serial.print(" m"); // distance measurement unit
}
// LED blinks to indicate device connection
void newDevice(DW1000Device *device)
{
    // connection with a new uwb sensor device 
    Serial.print("ranging init: 1 device added -> ");
    Serial.print("\tdevice short address: \n");
    Serial.print(dev->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{

    // debugging purpose, prints inactivity 
    Serial.print("delete inactive device: ");
    Serial.print("\tInactive device short address: \n");
    Serial.print(dev->getShortAddress(), HEX);
}
