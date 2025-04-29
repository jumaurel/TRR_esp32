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
    pinMode(LINE_COLOR_READ_PIN, INPUT);
    pinMode(LINE_COLOR_SELECT_PIN, OUTPUT);
    digitalWrite(LINE_COLOR_SELECT_PIN, LOW);
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

    globalState.leftDistance = static_cast<int16_t>(rawLeftDist);
    globalState.rightDistance = static_cast<int16_t>(rawRightDist);
    /*Serial.print(globalState.leftDistance);
    Serial.print(" - ");
    Serial.println(globalState.rightDistance);*/

    globalState.lineDetected = line;

}


// examples of readings:
//
// Ground color | red | blue
// -------------|-----|-----
// Red          |  17 |  41
// Red + sun    |  16 |  37
// Gray         |  20 |  20
// Gray + sun   |   6 |   6
// Black        | 120 | 120
// Black + sun  |  20 |  35
// White        |  15 |  13
// White + sun  |   4 |   4
// Blue         |  65 |  25
// Blue + sun   |  15 |  15
//
// We return the % of blue in the sum, so numbers
// significantly above .5 are blue
// and numbers significantly below .5 are red.
//
float SensorManager::readLineColor() {
    digitalWrite(LINE_COLOR_SELECT_PIN, LOW);
    int blue = pulseIn(LINE_COLOR_READ_PIN, HIGH);
    digitalWrite(LINE_COLOR_SELECT_PIN, HIGH);
    int red = pulseIn(LINE_COLOR_READ_PIN, HIGH);

    char buffer[30];
    snprintf(buffer, sizeof(buffer), "Read red: %d, blue %d", red, blue);
    Serial.println(buffer);

    return float(blue) / float(red + blue);
}

float SensorManager::readDistance(uint8_t pin, float& lastValidDist) {
    const uint16_t MAX_PULSE_WIDTH = 1850;
    const uint16_t MIN_PULSE_WIDTH = 1000;

    int16_t t = pulseIn(pin, HIGH);

    // Check if reading is valid
    if (t <= MAX_PULSE_WIDTH && t >= MIN_PULSE_WIDTH) {
        // Convert pulse width to distance using manufacturer's formula
        float d = (t - MIN_PULSE_WIDTH) * 2;
        // Ensure distance is not negative
        return d > 0 ? d : 0.0;
    }
    else if(t > MAX_PULSE_WIDTH){
        return 1000;
    }
    else{
        return lastValidDist;
    }

}