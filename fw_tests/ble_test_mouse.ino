#include <BleConnectionStatus.h>
#include <BleMouse.h>

#define comm_data_rate 115200
#define comm_init_delay 1000

#define SCROLL_THRES 100000
#define CLICK_THRES 100

// global variables to keep track of current screen coordinates
float curr_x, curr_y;
float prev_x, prev_y;

// bluetooth mouse instance
BleMouse bleMouse;

bool PEN_BUTTON = false;
bool is_pen_pressed = 0;
unsigned long pen_button_start_time = 0;
unsigned long pen_button_pressed_time;

void setup()
{
    Serial.begin(comm_data_rate);
    delay(comm_init_delay);
   
    // BLE HID init
    curr_x = 0;
    curr_y = 0;
    prev_x = 0;
    prev_y = 0;
    pen_button_pressed_time = millis();
//    BLEDevice::init("ESP32-BLE-MOUSE-TEST");
    bleMouse.begin();
}

void loop()
{
    // process these data to replicate bluetooth HID        // Bluetooth HID emulation here, use the coordinates received after localization
    if (bleMouse.isConnected()) {
        Serial.println("Connected to BLE Mouse");
        send_mouse_emulation_test_v2();
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
}

void send_mouse_emulation_test_v2() {

    int delta_cursor_x, delta_cursor_y;
    int scroll_amount = 10; // default, test via visual feedback
    unsigned long pen_button_time;

    unsigned long press_duration;

    // check if button has been depressed for the first time, if yes, keep track of when.
    pen_button_time = millis() - pen_button_pressed_time;
    if (pen_button_time >= 10000) {
      PEN_BUTTON = true;
    }
    if ((PEN_BUTTON) == false) {
        if(is_pen_pressed == false) {
            pen_button_start_time = millis();
            is_pen_pressed = true;
        }
        press_duration = millis() - pen_button_start_time;
        delta_cursor_x = 10;
        delta_cursor_y = 10;

        // differentiate between scrolling, clicking, and moving
        if (press_duration >= SCROLL_THRES) {
            // implement scrolling here
            // differentiate between scroll up and down
            if (delta_cursor_y > 0) {
                // scroll up
                bleMouse.move(0, 0, scroll_amount);
                delay(10);
//                PEN_BUTTON = true;
            }
            else if (delta_cursor_y < 0) {
                // scroll down
                bleMouse.move(0, 0, -scroll_amount);
                delay(10);
//                PEN_BUTTON = true;
            }

            // else, static hold, no need to scroll

        }
        else {
            // implement smooth movement of the cursor, until the pen button is high
            bleMouse.move(delta_cursor_x, delta_cursor_y);
            delay(10);
//            PEN_BUTTON = true;
        }
    }
//    else if ((PEN_BUTTON) == true && is_pen_pressed) {
//        press_duration = millis() - pen_button_start_time;
//
//        if (press_duration < CLICK_THRES){
//            // implement clicking here
//            bleMouse.click(MOUSE_LEFT);
//            delay(10); // delay to allow the host device react upon the Bluetooth HID command
//        }
//
//        is_pen_pressed = false;
//    }

}