#include <BLEMouse.h>

#define comm_data_rate 115920
#define comm_init_delay 1000

// global variables to keep track of current screen coordinates
float curr_x, curr_y; 
float prev_x, prev_y; 

// bluetooth mouse instance
BLEMouse bleMouse;

void setup()
{
    Serial.begin(comm_data_rate);
    delay(comm_init_delay);
    
    // BLE HID init
    curr_x = 0; 
    curr_y = 0;
    prev_x = 0;
    prev_y = 0; 
    BLEDevice::init("ESP32-BLE-MOUSE-TEST");
    bleMouse.begin();
}

void loop()
{
    // process these data to replicate bluetooth HID        // Bluetooth HID emulation here, use the coordinates received after localization
    if (bleMouse.isConnected()) {
        Serial.println("Connected to BLE Mouse");
        send_mouse_emulation_test(); 
        // CONSIDER DELAYS, use visual feedback to see if there are lags due to host device being overwhelmed 
    }
}

// test
void send_mouse_emulation_test() {
    unsigned long prev_time;

    Serial.println("move mouse up");
    prev_time = millis();
    while (millis() - prev_time < 2000) {
        bleMouse.move(0, 1); // testing move cursor up
        delay(100);
    }
    Serial.println("move mouse down");
    prev_time = millis();
    while (millis() - prev_time < 2000) {
        bleMouse.move(0, -1); // testing move cursor up
        delay(100);
    }
    Serial.println("move mouse left");
    prev_time = millis();
    while (millis() - prev_time < 2000) {
        bleMouse.move(-1, 0); // testing move cursor up
        delay(100);
    }
    Serial.println("move mouse right");
    prev_time = millis();
    while (millis() - prev_time < 2000) {
        bleMouse.move(1, 0); // testing move cursor up
        delay(100);
    }
    Serial.println("scroll up");
    prev_time = millis();
    while (millis() - prev_time < 2000) {
        bleMouse.move(0, 0, 1); // testing move cursor up
        delay(100);
    }
    Serial.println("scroll down");
    prev_time = millis();
    while (millis() - prev_time < 2000) {
        bleMouse.move(0, 0, -1); // testing move cursor up
        delay(100);
    }

    Serial.println("click");
    bleMouse.click(MOUSE_LEFT);
    delay(100);
}