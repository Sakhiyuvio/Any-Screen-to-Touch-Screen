#include <BLEMouse.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define comm_data_rate 115920
#define comm_init_delay 1000
#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1"

// BLE setup
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool host_dev_connected = false;

// global variables to keep track of current screen coordinates
float curr_x, curr_y; 
float prev_x, prev_y; 

// bluetooth mouse instance
BLEMouse bleMouse;

void setup()
{
    Serial.begin(comm_data_rate);
    delay(comm_init_delay);
    
    // BLE dev and HID init
    curr_x = 0; 
    curr_y = 0;
    prev_x = 0;
    prev_y = 0; 
    BLEDevice::init("ESP32-BLE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    // BLE service
    BLEService *pService = pServer->createService(SERVICE_UUID); // service uuid
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID, // char uuid
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );

    pService->start();
    pServer->getAdvertising()->start();
    Serial.println("BLE Waiting for host device Connection...");
    bleMouse.begin();
}

void loop()
{
    // add logic, set a timeout based protocol to send data 
    // and signal to host device for monitoring
    if (host_dev_connected) {
        Serial.println("BLE connected to host device");

        // optional in-depth testing 
        // send binary buffer
        uint8_t data[8]; // per byte
        memcpy(data, &curr_x, sizeof(float));
        memcpy(data + sizeof(float), &curr_y, sizeof(float));
        pCharacteristic->setValue(data, 8);
        pCharacteristic->notify();
    }

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

// BLE callback

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect (BLEServer* pServer) {
        host_dev_connected = true;
        Serial.println("Connected to host device");
    }

    void onDisconnect (BLEServer* pServer) {
        host_dev_connected = false;
        Serial.println("Disconnected from host device");
    }
};