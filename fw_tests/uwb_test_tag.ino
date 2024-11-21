
// include necessary libraries 

#include <SPI.h> 
#include "DW1000Ranging.h"


// ESP32-S3 SPI pin config
#define SPI_SCLK 12
#define SPI_MISO 13
#define SPI_MOSI 11
#define SPI_CS 10

// data rate 
#define comm_data_rate 115200 // communication of dwm1000 > 110kbps, stretch a bit higher for delay and accuracy purposes
#define comm_init_delay 1000 // 1 s 

#define ANCHOR_ADDR_1 0x82
#define ANCHOR_ADDR_2 0x84

// DWM1000 pin config
// ranging library utilizes uint8_t for parameters
const uint8_t RST_pin = 5;
const uint8_t CS_pin = 10;
const uint8_t INT_pin = 1;

char pen_uwb_addr[] = "7D:00:22:EA:82:60:3B:9C"; // unique addr

int anchor_one_flag, anchor_two_flag; // for uwb ranging handler


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

    // filter for better accuracy 
    //DW1000Ranging.useRangeFilter(true);

    // LED for debugging 
    // DW1000.enableDebounceClock();
    // DW1000.enableLedBlinking();
    // enables GPIO0/RXOKLED (Pin 15 in DWM1000, Pin 38 in DW1000)
    //for a short duration on reception of a frame with good CRC.
    // DW1000.setGPIOMode(MSGP0, LED_MODE);

    // dsp pipeline here, filtering dist data and/or improve accuracy

    // start dwm as tag, set flag to 0 
    anchor_one_flag = 0;
    anchor_two_flag = 0; 
    DW1000Ranging.startAsTag(pen_uwb_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);    
}

// continuous looping for uwb real-time data processing 
void loop()
{
    // continuously loop to gather ranging data, 
    // this function handles the Two-Way Ranging Algorithm to accurately measure distance 
    // between anchor and pen. 
    DW1000Ranging.loop();

    if (anchor_one_flag && anchor_two_flag) {
        Serial.println("Ranging from two anchors received, proceed to localize the coordinates!");
    }
    else {
        Serial.println("Multi-ranging failed");
    }
}

// handler functions
void newRange()
{
    // seek the anchor addresses
    // while not found
    anchor_one_flag = 0;
    anchor_two_flag = 0; 
    uint16_t device_addr; 

    /* 
    TO DO: 
    Instead of only printing, store and send data to host device for HID (MUTHU)
    */

    // loop until both the anchor devices are found 
    // curr_device = DW1000Ranging.getDistantDevice();
    device_addr = DW1000Ranging.getDistantDevice()->getShortAddress();

    if(device_addr == ANCHOR_ADDR_1 && !anchor_one_flag) {
        // We know data is from anchor one

        // send data for localization
        curr_range_uwb_1 = DW1000Ranging.getDistantDevice()->getRange();  

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
        curr_range_uwb_2 = DW1000Ranging.getDistantDevice()->getRange();  

        // PRINT FOR TESTING
        Serial.print("Data from anchor one: ");
        Serial.print("\nCurrent Range: ");
        Serial.print(curr_range_uwb_2);
        Serial.print(" m");
        // set flag
        anchor_two_flag = 1;    
    }
}
// LED blinks to indicate device connection
void newDevice(DW1000Device *device)
{
    // connection with a new uwb sensor device 
    Serial.print("ranging init; 1 device added ! -> ");
    Serial.print(" short:");
    Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
    // debugging purpose, prints inactivity 
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);
}
