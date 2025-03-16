#pragma once

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

class BLEManager {
public:
    static void setup();
    static void sendSensorData();
    static bool isConnected() { return deviceConnected; }
    
private:
    static BLEServer* pServer;
    static BLECharacteristic* pControlCharacteristic;
    static BLECharacteristic* pSensorCharacteristic;
    static bool deviceConnected;
    
    class ServerCallbacks;
    class CharacteristicCallbacks;
    
    static void processControlCommand(uint8_t* data, size_t length);
}; 