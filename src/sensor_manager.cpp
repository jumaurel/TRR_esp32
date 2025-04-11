#include "sensor_manager.h"
#include "config.h"
#include "state.h"
#include <Arduino.h>

// Static variables to store last valid distances and error counts
static float lastValidLeftDistance = 0.0;
static float lastValidRightDistance = 0.0;

// EMA filter parameters
static const float ALPHA = 0.3;  // Smoothing factor (0-1), lower = more smoothing
static float filteredLeftDistance = 0.0;
static float filteredRightDistance = 0.0;

// Maximum allowed change between consecutive readings (in mm)
static const float MAX_DISTANCE_CHANGE = 500.0;  // Increased to allow for faster movements

void SensorManager::setup() {
    pinMode(LEFT_SENSOR_PIN, INPUT);
    pinMode(RIGHT_SENSOR_PIN, INPUT);
    pinMode(LINE_SENSOR_PIN, INPUT);
}

void SensorManager::update() {
    // Read sensors with error handling
    float rawLeftDist = readDistance(LEFT_SENSOR_PIN, lastValidLeftDistance);
    float rawRightDist = readDistance(RIGHT_SENSOR_PIN, lastValidRightDistance);
    
    // Apply constraints (0 to 1000mm)
    rawLeftDist = constrain(rawLeftDist, 0.0, 1000.0);
    rawRightDist = constrain(rawRightDist, 0.0, 1000.0);
    
    // Check for unrealistic changes
    if (abs(rawLeftDist - filteredLeftDistance) > MAX_DISTANCE_CHANGE) {
        rawLeftDist = filteredLeftDistance;  // Reject the new reading if change is too large
    }
    if (abs(rawRightDist - filteredRightDistance) > MAX_DISTANCE_CHANGE) {
        rawRightDist = filteredRightDistance;  // Reject the new reading if change is too large
    }
    
    // Apply EMA filter
    filteredLeftDistance = ALPHA * rawLeftDist + (1 - ALPHA) * filteredLeftDistance;
    filteredRightDistance = ALPHA * rawRightDist + (1 - ALPHA) * filteredRightDistance;
    
    // Update last valid distances
    lastValidLeftDistance = filteredLeftDistance;
    lastValidRightDistance = filteredRightDistance;
    
    // Read line sensor
    bool line = digitalRead(LINE_SENSOR_PIN);
    
    // Update global state with filtered values
    globalState.leftDistance = static_cast<int16_t>(filteredLeftDistance);
    globalState.rightDistance = static_cast<int16_t>(filteredRightDistance);
    globalState.lineDetected = line;

    // Debug output
    /*Serial.print("Left: ");
    Serial.print(filteredLeftDistance);
    Serial.print(" - Right: ");
    Serial.println(filteredRightDistance);*/
}

float SensorManager::readDistance(uint8_t pin, float& lastValidDist) {
    const uint16_t MAX_PULSE_WIDTH = 1850;
    const uint16_t MIN_PULSE_WIDTH = 1000;
    
    int16_t t = pulseIn(pin, HIGH);
        
    // Check if reading is valid
    if (t > 0 && t <= MAX_PULSE_WIDTH && t >= MIN_PULSE_WIDTH) {
        // Convert pulse width to distance using manufacturer's formula
        float d = (t - MIN_PULSE_WIDTH) * 2;
        // Ensure distance is not negative
        return d > 0 ? d : 0.0;           
    }
    
    // If we get here, all attempts failed
    return lastValidDist;  // Return last valid distance
}