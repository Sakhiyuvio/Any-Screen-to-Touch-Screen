// include necessary libraries

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

#define ANCHOR_ADDR_1 "82:17:5B:D5:A9:9A:E2:9C" // use reference from makerlab for now
#define ANCHOR_ADDR_2 "84:17:5B:D5:A9:9A:E2:9C" // use reference from makerlab for now

#define PEN_UWB_ADDR 0xE29A // pen short address

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

float known_target_dist = 0.2; // change to actual measured distance in meter

uint16_t curr_antenna_delay = 16600;
uint16_t binary_search_delta = 100; // binary search step size

// handler functions
void ranging_handler(){
    // get info of tag addr, only perform ranging if talking to pen uwb
    static float last_delta = 0.0;

    Serial.print("Data from: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX); // print hex address

    float recorded_dist = DW1000Ranging.getDistantDevice()->getRange();

    Serial.print("\nCurrent Range: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRange());
    Serial.print(" m"); // distance measurement unit

    if (binary_search_delta < 3) {
        Serial.print(", final Adelay = ");
        Serial.println(curr_antenna_delay);
        while(1); // done, get the antenna delay and set it for the anchors individually
    }

    float new_delta = recorded_dist - known_target_dist;

    if (new_delta * last_delta < 0.0) {
        binary_search_delta = binary_search_delta / 2;
    }

    last_delta = new_delta;
    if (new_delta > 0.0) {
        curr_antenna_delay += binary_search_delta;
    }
    else {
        curr_antenna_delay -= binary_search_delta;
    }

    Serial.print(", Adelay = ");
    Serial.println(curr_antenna_delay);
    DW1000.setAntennaDelay(curr_antenna_delay);

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
    Serial.print("\tInactive device short address: \n");
    Serial.print(dev->getShortAddress(), HEX);
}

void setup()
{
    // start communication protocol initialization
    Serial.begin(comm_data_rate);
    delay(comm_init_delay);

    // SPI and DWM1000 initialization
    SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, SPI_CS);
    DW1000Ranging.initCommunication(RST_pin, CS_pin, INT_pin);

    // setup initial antenna delay
    Serial.print("Starting Antenna Delay");
    DW1000.setAntennaDelay(curr_antenna_delay);

    // create and call handlers here, to get ranging data, LED, and device activation
    DW1000Ranging.attachNewRange(ranging_handler); // process distance data between anchor and pen
    DW1000Ranging.attachBlinkDevice(new_dev_handler);
    DW1000Ranging.attachInactiveDevice(inactive_handler);

    // dsp pipeline here, filtering dist data and/or improve accuracy

    // start dwm as an anchor module, first/second depends on calibrating process

    DW1000Ranging.startAsAnchor(ANCHOR_ADDR_1, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
    // DW1000Ranging.startAsAnchor(ANCHOR_ADDR_2, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);    
   
}

// continuous looping for uwb real-time data processing
void loop()
{
    // continuously loop to gather ranging data,
    // this function handles the Two-Way Ranging Algorithm to accurately measure distance
    // between anchor and pen.
    DW1000Ranging.loop();
}