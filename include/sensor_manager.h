#pragma once

#include <Arduino.h>
#include "config.h"

class SensorManager {
public:
    // Constructor replaces setup
    SensorManager();

    void update();
    float readDistance(uint8_t pin, float& lastValidDist);

private:
};