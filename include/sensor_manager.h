#pragma once

#include <Arduino.h>

class SensorManager {
public:
    static void setup();
    static void update();
private:
    static int16_t readDistance(uint8_t pin);
    static int16_t fakeRandomRightDistance(uint8_t pin);
    static int16_t fakeRandomLeftDistance(uint8_t pin);
};
