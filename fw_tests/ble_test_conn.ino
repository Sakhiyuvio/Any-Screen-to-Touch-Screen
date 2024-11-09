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

void setup()
{
    Serial.begin(comm_data_rate);
    delay(comm_init_delay);
    
    // BLE dev and HID init
    curr_x = 0; 
    curr_y = 0;
    prev_x = 0;
    prev_y = 0; 
    BLEDevice::init("ESP32-BLE-TEST");
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