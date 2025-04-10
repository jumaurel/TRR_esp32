#include "sensor_manager.h"
#include "config.h"
#include "state.h"
#include <Arduino.h>

// Static variables to store last valid distances and error counts
static int16_t lastValidLeftDistance = 400;  // Default to middle range
static int16_t lastValidRightDistance = 400; // Default to middle range

void SensorManager::setup() {
    pinMode(LEFT_SENSOR_PIN, INPUT);
    pinMode(RIGHT_SENSOR_PIN, INPUT);
    pinMode(LINE_SENSOR_PIN, INPUT);
}

void SensorManager::update() {
    // Read sensors with error handling
    int16_t rawLeftDist = readDistance(LEFT_SENSOR_PIN, lastValidLeftDistance);
    int16_t rawRightDist = readDistance(RIGHT_SENSOR_PIN, lastValidRightDistance);
    
    // Apply constraints
    rawLeftDist = constrain(rawLeftDist, 50, 800);
    rawRightDist = constrain(rawRightDist, 50, 800);
    
    // Update last valid distances
    lastValidLeftDistance = rawLeftDist;
    lastValidRightDistance = rawRightDist;
    
    // Read line sensor
    bool line = digitalRead(LINE_SENSOR_PIN);
    
    // Update global state
    globalState.leftDistance = rawLeftDist;
    globalState.rightDistance = rawRightDist;
    globalState.lineDetected = line;

    // Debug output
    /*Serial.print("Left: ");
    Serial.print(rawLeftDist);
    Serial.print(" - Right: ");
    Serial.print(rawRightDist);*/

}

int16_t SensorManager::readDistance(uint8_t pin, int16_t& lastValidDist) {
    // Try to read the sensor multiple times, if it fails, return the last valid distance
    const uint8_t MAX_READ_ATTEMPTS = 3;
    const uint16_t MAX_PULSE_WIDTH = 1850;
    const uint16_t MIN_PULSE_WIDTH = 1000;
    const int16_t MAX_DISTANCE_CHANGE = 100; // Maximum allowed change in mm between readings
    
    for (uint8_t attempt = 0; attempt < MAX_READ_ATTEMPTS; attempt++) {
        int16_t t = pulseIn(pin, HIGH);
        
        // Check if reading is valid
        if (t > 0 && t <= MAX_PULSE_WIDTH && t >= MIN_PULSE_WIDTH) {
            // Convert pulse width to distance
            int16_t d = (t - MIN_PULSE_WIDTH) * 2;
            if (d >= 0) {
                // Check if the new distance is not too far from the last valid one
                if (abs(d - lastValidDist) <= MAX_DISTANCE_CHANGE) {
                    return d;
                }
            }
        }
        
        // Small delay between attempts
        delay(1);
    }
    
    // If we get here, all attempts failed
    return lastValidDist;  // Return last valid distance
}