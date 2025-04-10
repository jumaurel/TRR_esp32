#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include "config.h"

class SensorManager {
public:
    static void setup();
    static void update();
    
private:
    static int16_t readDistance(uint8_t pin, int16_t& lastValidDist);
    static int16_t calculateAverage(int16_t* readings, uint8_t numReadings);
};

#endif