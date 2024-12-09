// include necessary libraries

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include <Wire.h>
#include <SparkFunBQ27441.h>

// Set BATTERY_CAPACITY to the design capacity of your battery.
const unsigned int BATTERY_CAPACITY = 3500; // e.g. 850mAh battery
#define SDA_PIN 1
#define SCL_PIN 2
#define GPOUT_PIN 21

#define ANCHOR_ADDR_1 "82:17:5B:D5:A9:9A:E2:9C" // use reference from makerlab for now
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

uint16_t calibrated_antenna_delay = 16609; // antenna delay post-calibration

unsigned long lastBatteryPrintTime = 0;
const unsigned long batteryPrintInterval = 5000;

// handler functions
void ranging_handler()
{
    // get info of tag addr, only perform ranging if talking to pen uwb
    Serial.print("Data from: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX); // print hex address
    Serial.print("\nCurrent Range: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRange());
    Serial.print(" m"); // distance measurement unit
   
//    else {
//        Serial.print("Data Invalid (Unknown)");
//    }
    return;
    // might need to analyze signal power here, skip for now
}

// LED blinks to indicate device connection
void LED_handler(DW1000Device *dev)
{
    // connection with a new uwb sensor device
    Serial.print("LED blink, a new device is connected!");
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

    Wire.begin(SDA_PIN, SCL_PIN);
    if (!lipo.begin()) // begin() will return true if communication is successful
    {
    // If communication fails, print an error message and loop forever.
      Serial.println("Error: Unable to communicate with BQ27441.");
      Serial.println("  Check wiring and try again.");
      Serial.println("  (Battery must be plugged into Battery Babysitter!)");
      while (1) ;
    }
    Serial.println("Connected to BQ27441!");
    
    // Uset lipo.setCapacity(BATTERY_CAPACITY) to set the design capacity
    // of your battery.
    lipo.setCapacity(BATTERY_CAPACITY);
    printBatteryStats();
    delay(6000);
    printBatteryStats();
    // SPI and DWM1000 initialization
    SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, SPI_CS);
    DW1000Ranging.initCommunication(RST_pin, CS_pin, INT_pin);

    DW1000.setAntennaDelay(calibrated_antenna_delay);

    // create and call handlers here, to get ranging data, LED, and device activation
    DW1000Ranging.attachNewRange(ranging_handler); // process distance data between anchor and pen
    DW1000Ranging.attachBlinkDevice(LED_handler);
    DW1000Ranging.attachInactiveDevice(inactive_handler);

    // dsp pipeline here, filtering dist data and/or improve accuracy

    // start dwm as an anchor module, second anchor
    DW1000Ranging.startAsAnchor(ANCHOR_ADDR_1, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);    
}

// continuous looping for uwb real-time data processing
void loop()
{
    // Continuously run the DW1000 ranging algorithm
    DW1000Ranging.loop();

    // Check if it's time to print battery stats
    unsigned long currentTime = millis();
    print(currentTime)
    if (currentTime - lastBatteryPrintTime >= batteryPrintInterval) {
        lastBatteryPrintTime = currentTime; // Update the last print time
        printBatteryStats(); // Print the battery stats
    }
}



void printBatteryStats()
{
  // Read battery stats from the BQ27441-G1A
  unsigned int soc = lipo.soc();  // Read state-of-charge (%)
  float volts = (float(lipo.voltage()))/1000; // Read battery voltage (mV)
  int current = lipo.current(AVG); // Read average current (mA)
  unsigned int fullCapacity = lipo.capacity(FULL); // Read full capacity (mAh)
  unsigned int capacity = lipo.capacity(REMAIN); // Read remaining capacity (mAh)
  int power = lipo.power(); // Read average power draw (mW)
  int health = lipo.soh(); // Read state-of-health (%)

  // Now print out those values:
  String toPrint = String(soc) + "% | ";
  toPrint += String(volts) + " V | ";
  toPrint += String(current) + " mA | ";
  toPrint += String(capacity) + " / ";
  toPrint += String(fullCapacity) + " mAh | ";
  toPrint += String(power) + " mW | ";
  toPrint += String(health) + "%";
  
  Serial.println(toPrint);
}