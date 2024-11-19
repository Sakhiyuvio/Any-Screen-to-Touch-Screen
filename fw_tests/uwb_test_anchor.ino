
// include necessary libraries 

#include <SPI.h> 
#include "DW1000Ranging.h"
#include "DW1000.h"

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

char anchor_uwb_addr[] = "84:00:5B:D5:A9:9A:E2:9C"; // unique addr

//calibrated Antenna Delay setting for this anchor
uint16_t Adelay = 16580;

void setup()
{
    // start communication protocol initialization
    Serial.begin(comm_data_rate);
    delay(comm_init_delay);

    // SPI and DWM1000 initialization
    SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(RST_pin, CS_pin, INT_pin);

    // set antenna delay for anchors only. Tag is default (16384)
    // DW1000.setAntennaDelay(Adelay);

    // create and call handlers here, to get ranging data, LED, and device activation
    DW1000Ranging.attachNewRange(newRange); // process distance data between anchor and pen
    DW1000Ranging.attachNewDevice(newDevice);
    // DW1000Ranging.attachBlinkDevice(newBlink); 
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    //DW1000Ranging.useRangeFilter(true);

    // start dwm as an anchor module, first anchor
    DW1000Ranging.startAsAnchor(anchor_uwb_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);    
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
    Serial.print("from: "); Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
    Serial.print("\t Range: "); Serial.print(DW1000Ranging.getDistantDevice()->getRange()); Serial.print(" m");
    Serial.print("\t RX power: "); Serial.print(DW1000Ranging.getDistantDevice()->getRXPower()); Serial.println(" dBm");
}

void newDevice(DW1000Device *device)
{
    Serial.print("Device added: ");
    Serial.println(device->getShortAddress(), HEX);
}

// LED blinks to indicate device connection
void newBlink(DW1000Device *device)
{
    // connection with a new uwb sensor device 
    Serial.print("blink; 1 device added ! -> ");
    Serial.print(" short:");
    Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{

    // debugging purpose, prints inactivity 
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);
}
