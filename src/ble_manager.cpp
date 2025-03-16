#include "ble_manager.h"
#include "pid_controller.h"
#include "config.h"
#include "state.h"
#include "main.h" 
#include <Arduino.h>

// Static member initialization
BLEServer* BLEManager::pServer = nullptr;
BLECharacteristic* BLEManager::pControlCharacteristic = nullptr;
BLECharacteristic* BLEManager::pSensorCharacteristic = nullptr;
bool BLEManager::deviceConnected = false;

// Server callbacks implementation
class BLEManager::ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
        Serial.println("Client connected!");
        Serial.print("Number of connected clients: ");
        Serial.println(pServer->getConnectedCount());
    }

    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        Serial.println("Client disconnected!");
        Serial.println("Restarting advertising...");
        BLEDevice::startAdvertising();
    }
};

// Characteristic callbacks implementation
class BLEManager::CharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        
        std::string value = pCharacteristic->getValue();
        Serial.print("Received command: ");
        for(int i = 0; i < value.length(); i++) {
            Serial.print((uint8_t)value[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        
        if (value.length() > 0) {
            processControlCommand((uint8_t*)value.c_str(), value.length());
        }
    }
};

void BLEManager::setup() {
    Serial.println("Starting BLE setup...");
    
    // Initialize BLE
    BLEDevice::init("TRR_CAR");
    
    // Set power to maximum
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    Serial.println("BLE Device initialized with max power");
    
    // Create server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    // Create service
    BLEService* pService = pServer->createService(BLEUUID(SERVICE_UUID));
    
    // Create characteristics
    pControlCharacteristic = pService->createCharacteristic(
        BLEUUID(CONTROL_CHAR_UUID),
        BLECharacteristic::PROPERTY_WRITE
    );
    pSensorCharacteristic = pService->createCharacteristic(
        BLEUUID(SENSOR_CHAR_UUID),
        BLECharacteristic::PROPERTY_NOTIFY
    );
    
    // Create a BLE Descriptor for notifications
    pSensorCharacteristic->addDescriptor(new BLE2902());
    
    // Set callbacks
    pControlCharacteristic->setCallbacks(new CharacteristicCallbacks());
    
    // Start the service
    pService->start();
    
    // Start advertising
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLEUUID(SERVICE_UUID));
    pAdvertising->setScanResponse(true);
    
    // Configuration simple mais efficace
    pAdvertising->setMinInterval(0x20);  // 32ms
    pAdvertising->setMaxInterval(0x40);  // 64ms
    
    // Démarrer l'advertising
    BLEDevice::startAdvertising();
    
    Serial.println("BLE Setup complete with:");
    Serial.print("Name: TRR_CAR\n");
    Serial.print("Service UUID: ");
    Serial.println(SERVICE_UUID);
    Serial.print("Control Char UUID: ");
    Serial.println(CONTROL_CHAR_UUID);
    Serial.print("Sensor Char UUID: ");
    Serial.println(SENSOR_CHAR_UUID);
    Serial.println("Advertising started...");
}

void BLEManager::sendSensorData() {
    if (!deviceConnected) return;
    
    uint8_t sensorData[5];
    sensorData[0] = globalState.leftDistance & 0xFF;
    sensorData[1] = (globalState.leftDistance >> 8) & 0xFF;
    sensorData[2] = globalState.rightDistance & 0xFF;
    sensorData[3] = (globalState.rightDistance >> 8) & 0xFF;
    sensorData[4] = globalState.lineDetected ? 1 : 0;
    
    pSensorCharacteristic->setValue(sensorData, 5);
    pSensorCharacteristic->notify();
} 

void BLEManager::processControlCommand(uint8_t* data, size_t length) {
  if (length < 2) return;
  
  if (xSemaphoreTake(globalMutex, portMAX_DELAY)) {
    switch (data[0]) {
      case 1: // Motor 1
        globalState.motor1Speed = data[1];
        break;
      case 2: // Motor 2
        globalState.motor2Speed = data[1];
        break;
      case 3: // Direction
        globalState.servoAngle = data[1] - 45; // Convert 0-90 to -45-45
        break;
      case 4: // Mode
        globalState.isAutoMode = data[1] == 1;
        if (globalState.isAutoMode) {
          // Reset PID variables
            PIDController::reset();
        }
        break;
      case 5: // Emergency stop
        globalState.emergency = true;
        globalState.motor1Speed = 0;
        globalState.motor2Speed = 0;
        break;
      default:
        Serial.println("Unknown command received");
        break;
    }
    xSemaphoreGive(globalMutex);
  }
} 