#pragma once

#include <Arduino.h>
#include "config.h"

class SensorManager {
public:
    static void setup();
    static void update();
    static float readDistance(uint8_t pin, float& lastValidDist);
    static int readLineColor();
    static void checkTrackColor();
    static void trackCounter();
    static void readFrontSensor();
};