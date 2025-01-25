#include "config.h"
/*      Bluetooth Setup   */
bool deviceConnected;
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};
BLEServer* pServer;
BLEService* pService;
BLECharacteristic* pCharacteristic;

void initBLE() {
  BLEDevice::init("ESP32_Electrolyser_BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                     CHARACTERISTIC_UUID,
                     BLECharacteristic::PROPERTY_READ |
                     BLECharacteristic::PROPERTY_WRITE |
                     BLECharacteristic::PROPERTY_NOTIFY
                   );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
}

void sendFileOverBluetooth(const char *path) {
    Serial.printf("Reading file: %s\n", path);
    File file = SD.open(path, FILE_READ);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return;
    }

    const int bufferSize = 512;
    uint8_t buffer[bufferSize];
    size_t bytesRead;

    while ((bytesRead = file.read(buffer, bufferSize)) > 0) {
        if (deviceConnected) {
            // Send data in chunks
            for (size_t i = 0; i < bytesRead; i += 20) {
                size_t chunkSize = min(20, (int)(bytesRead - i));
                pCharacteristic->setValue(&buffer[i], chunkSize);
                pCharacteristic->notify();
                delay(10); // Give client time to process
            }
        }
    }

    file.close();
    Serial.println("File sent over BLE");
}