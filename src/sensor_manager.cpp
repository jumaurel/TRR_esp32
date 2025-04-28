#include "sensor_manager.h"
#include "config.h"
#include "motor_control.h"
#include "sensor_manager.h"
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

extern SensorData sensorData;

SensorManager::SensorManager() {
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

    // Read line sensor
    bool line = digitalRead(LINE_SENSOR_PIN);

    // Update global state with filtered values

    /*if(abs(rawLeftDist - lastValidLeftDistance) > 200){
        rawLeftDist = lastValidLeftDistance;
    }

    if(abs(rawRightDist - lastValidRightDistance) > 200){
        rawRightDist = lastValidRightDistance;
    }*/

     // Update last valid distances
    lastValidLeftDistance = rawLeftDist;
    lastValidRightDistance = rawRightDist;


    sensorData.leftDistance = static_cast<int16_t>(rawLeftDist);
    sensorData.rightDistance = static_cast<int16_t>(rawRightDist);
    /*Serial.print(globalState.leftDistance);
    Serial.print(" - ");
    Serial.println(globalState.rightDistance);*/

    sensorData.lineDetected = line;

}

float SensorManager::readDistance(uint8_t pin, float& lastValidDist) {
    const uint16_t MAX_PW = 1850;
    const uint16_t MIN_PW = 1000;

    int16_t t = pulseIn(pin, HIGH);

    // Check if reading is valid
    if (MIN_PW <= t && t <= MAX_PW) {
        // Convert pulse width to distance using manufacturer's formula
        float d = (t - MIN_PW) * 2;
        // Ensure distance is not negative
        return d > 0 ? d : 0.0;
    }
    else if(t > MAX_PW){
        return 1000;
    }
    else{
        return lastValidDist;
    }

}
