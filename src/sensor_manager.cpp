#include "sensor_manager.h"
#include "config.h"
#include "state.h"
#include <Arduino.h>

void SensorManager::setup() {
    pinMode(LEFT_SENSOR_PIN, INPUT);
    pinMode(RIGHT_SENSOR_PIN, INPUT);
    pinMode(LINE_SENSOR_PIN, INPUT);
}

void SensorManager::update() {
    int16_t leftDist = fakeRandomLeftDistance(LEFT_SENSOR_PIN);
    int16_t rightDist = fakeRandomRightDistance(RIGHT_SENSOR_PIN);
    bool line = digitalRead(LINE_SENSOR_PIN);
    
    globalState.leftDistance = leftDist;
    globalState.rightDistance = rightDist;
    globalState.lineDetected = line;
}

int16_t SensorManager::readDistance(uint8_t pin) {
    int16_t t = pulseIn(pin, HIGH);
    
    if (t == 0 || t > 1850) {
        return -1;  // Error or no detection
    }
    
    // Convert pulse width to distance
    int16_t d = (t - 1000) * 2;
    return d < 0 ? 0 : d;
} 

int16_t SensorManager::fakeRandomRightDistance(uint8_t pin) {
    return random(425, 450);
}

int16_t SensorManager::fakeRandomLeftDistance(uint8_t pin) {
    return random(425, 450);
}
