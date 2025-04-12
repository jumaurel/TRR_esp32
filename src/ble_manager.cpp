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
        Serial.println(pServer->getConnectedCount());
        Serial.print("Number of connected clients: ");
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
        //Serial.print("Received command: ");
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
    char commandType = (char)data[0];
    
    switch (commandType) {
      case 'M': // Motor control - Only allowed in auto mode
        if (globalState.isAutoMode) {
          Serial.println("Motor commands not allowed in auto mode");
          break;
        }
        if (data[1] == '1') { // Motor 1
          // Parse the value after M1
          int value = 0;
          for (size_t i = 2; i < length; i++) {
            if (isdigit(data[i])) {
              value = value * 10 + (data[i] - '0');
            }
          }
          globalState.motor1Speed = value;
          Serial.print("Motor 1 speed set to: ");
          Serial.println(value);
        } else if (data[1] == '2') { // Motor 2 
          // Parse the value after M2
          int value = 0;
          for (size_t i = 2; i < length; i++) {
            if (isdigit(data[i])) {
              value = value * 10 + (data[i] - '0');
            }
          }
          globalState.motor2Speed = value;
          Serial.print("Motor 2 speed set to: ");
          Serial.println(value);
        }
        break;
        
      case 'D': // Direction - Only allowed in auto mode
        if (globalState.isAutoMode) {
          Serial.println("Direction commands not allowed in auto mode");
          break;
        }
        {
          // Parse the signed integer value after D
          int value = 0;
          bool isNegative = false;
          size_t i = 1;
          
          if (i < length && data[i] == '-') {
            isNegative = true;
            i++;
          }
          
          for (; i < length; i++) {
            if (isdigit(data[i])) {
              value = value * 10 + (data[i] - '0');
            }
          }
          
          if (isNegative) {
            value = -value;
          }
          
          globalState.servoAngle = map(value, -30, 30, 52, 20); // Value is already in range -30 to 30
          Serial.print("Direction set to: ");
          Serial.println(value);
        }
        break;
        
      case 'A': // Auto mode
        globalState.isAutoMode = (data[1] == '1');
        if (globalState.isAutoMode) {
          // Reset PID variables
          PIDController::reset();
          Serial.println("Mode Auto enabled");
        } else {
          // Reset motor speeds when switching to manual mode
          globalState.motor1Speed = 0;
          globalState.motor2Speed = 0;
          Serial.println("Mode Manuel enabled");
        }
        break;
        
      case 'G': // Go
        if (data[1] == '1') {
          globalState.emergency = false;
          Serial.println("Go - Motors enabled");
        }
        break;
        
      case 'E': // Stop
        if (data[1] == '1') {
          globalState.emergency = true;
          globalState.motor1Speed = 0;
          globalState.motor2Speed = 0;
          Serial.println("Stop - Motors disabled");
        }
        break;
        
      case 'B': // Forward/Backward direction - Only allowed in auto mode
        if (globalState.isAutoMode) {
          Serial.println("Direction commands not allowed in auto mode");
          break;
        }
        if (data[1] == '0') {
          globalState.isForward = true;
          Serial.println("Direction set to Forward");
        } else if (data[1] == '1') {
          globalState.isForward = false;
          Serial.println("Direction set to Backward");
        }
        break;
        
      case 'T': // Power percentage in auto mode (0-100)
        if (!globalState.isAutoMode) {
          Serial.println("Power commands not allowed in manual mode");
          break;
        }
        {
          // Parse integer value (0-100)
          int value = 0;
          for (size_t i = 1; i < length; i++) {
            if (isdigit(data[i])) {
              value = value * 10 + (data[i] - '0');
            }
          }
          
          // Constrain value between 0 and 100
          value = constrain(value, 0, 100);
          
          // Convert percentage to motor speed (0-255)
          int motorSpeed = map(value, 0, 100, 0, 255);
          
          // Set auto mode motor speed in PID controller
          PIDController::setAutoModeMotorSpeed(motorSpeed);
          Serial.print("Power percentage: ");
          Serial.print(value);
          Serial.print("%, Auto mode motor speed: ");
          Serial.println(motorSpeed);
        }
        break;
        
      case 'P': // Kp parameter
        {
          // Parse floating-point value
          float value = 0.0;
          float decimal = 0.0;
          bool decimalPart = false;
          float decimalFactor = 0.1;
          
          for (size_t i = 1; i < length; i++) {
            if (data[i] == '.') {
              decimalPart = true;
            } else if (isdigit(data[i])) {
              if (decimalPart) {
                decimal += (data[i] - '0') * decimalFactor;
                decimalFactor *= 0.1;
              } else {
                value = value * 10 + (data[i] - '0');
              }
            }
          }
          
          value += decimal;
          PIDController::setKp(value);
          Serial.print("Kp set to: ");
          Serial.println(value);
        }
        break;
        
      case 'I': // Ki parameter
        {
          // Parse floating-point value
          float value = 0.0;
          float decimal = 0.0;
          bool decimalPart = false;
          float decimalFactor = 0.1;
          
          for (size_t i = 1; i < length; i++) {
            if (data[i] == '.') {
              decimalPart = true;
            } else if (isdigit(data[i])) {
              if (decimalPart) {
                decimal += (data[i] - '0') * decimalFactor;
                decimalFactor *= 0.1;
              } else {
                value = value * 10 + (data[i] - '0');
              }
            }
          }
          
          value += decimal;
          PIDController::setKi(value);
          Serial.print("Ki set to: ");
          Serial.println(value);
        }
        break;
        
      case 'K': // Kd parameter
        {
          // Parse floating-point value
          float value = 0.0;
          float decimal = 0.0;
          bool decimalPart = false;
          float decimalFactor = 0.1;
          
          for (size_t i = 1; i < length; i++) {
            if (data[i] == '.') {
              decimalPart = true;
            } else if (isdigit(data[i])) {
              if (decimalPart) {
                decimal += (data[i] - '0') * decimalFactor;
                decimalFactor *= 0.1;
              } else {
                value = value * 10 + (data[i] - '0');
              }
            }
          }
          
          value += decimal;
          PIDController::setKd(value);
          Serial.print("Kd set to: ");
          Serial.println(value);
        }
        break;
        
      default:
        Serial.print("Unknown command received: ");
        Serial.write(commandType);
        Serial.println();
        break;
    }
    xSemaphoreGive(globalMutex);
  }
} 