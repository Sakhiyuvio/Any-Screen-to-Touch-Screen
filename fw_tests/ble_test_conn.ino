#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define comm_data_rate 115200
#define comm_init_delay 1000
#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1"

// BLE setup
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool host_dev_connected = false;

float curr_x, curr_y;
unsigned long last_sent_time = 0;
unsigned long send_interval = 2000;

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
void setup()
{
    Serial.begin(comm_data_rate);
    delay(comm_init_delay);
   
    // BLE dev
    curr_x = 0;
    curr_y = 0;
    BLEDevice::init("ESP32");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    // BLE service
    BLEService *pService = pServer->createService(SERVICE_UUID); // service uuid
    pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
//    pServer->getAdvertising()->start();
    Serial.println("BLE Waiting for host device Connection...");
}

void loop()
{
    
    // add logic, set a timeout based protocol to send data
    // and signal to host device for monitoring
    if (host_dev_connected) {
        Serial.println("BLE connected to host device");

//        optional in-depth testing
//        send binary buffer
        if (millis() - last_sent_time >= send_interval) {
            uint8_t data[8]; // per byte
            memcpy(data, &curr_x, sizeof(float));
            memcpy(data + sizeof(float), &curr_y, sizeof(float));
            pCharacteristic->setValue(data, 8);
            pCharacteristic->notify();
            last_sent_time = millis(); 
            // Debugging info
            Serial.print("Data sent: X = ");
            Serial.print(curr_x);
            Serial.print(", Y = ");
            Serial.println(curr_y);
        }
    }
}