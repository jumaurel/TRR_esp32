#pragma once

#include <Arduino.h>

class SensorManager {
public:
    static void setup();
    static void update();
private:
    static int16_t readDistance(uint8_t pin);
    static int16_t calculateAverage(int16_t* readings, uint8_t numReadings);
};