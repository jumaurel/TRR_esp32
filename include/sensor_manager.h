#pragma once

#include <Arduino.h>
#include "config.h"

class SensorManager {
public:
    static void setup();
    static void update();
    static float readDistance(uint8_t pin, float& lastValidDist);
    
private:
    static int16_t calculateAverage(int16_t* readings, uint8_t numReadings);
};