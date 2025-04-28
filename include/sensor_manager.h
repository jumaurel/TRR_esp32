#pragma once

#include <Arduino.h>
#include "config.h"

// Struct to hold sensor-related data
struct SensorData {
    int16_t leftDistance;
    int16_t rightDistance;
    bool lineDetected;
    bool isAutoMode;
    int autoModeMotorSpeed;
};

class SensorManager {
public:
    // Constructor replaces setup
    SensorManager();

    void update();
    float readDistance(uint8_t pin, float& lastValidDist);

private:
};