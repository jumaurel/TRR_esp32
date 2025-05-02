#include "sensor_manager.h"
#include "config.h"
#include "state.h"
#include <Arduino.h>

// Static variables to store last valid distances and error counts
static float lastValidLeftDistance = 0.0;
static float lastValidRightDistance = 0.0;

// Circular buffer for track colors
static const int COLOR_BUFFER_SIZE = 5;
static int colorBuffer[COLOR_BUFFER_SIZE];
static int colorBufferIndex = 0;
static bool bufferInitialized = false;

// Static variables for track counting
static bool hasPassedStartLine = false;
static bool hasPassedFinishLine = false;
static int lastColor = 0;

void SensorManager::setup() {
    pinMode(LEFT_SENSOR_PIN, INPUT);
    pinMode(RIGHT_SENSOR_PIN, INPUT);
    pinMode(LINE_SENSOR_PIN, INPUT);
    pinMode(LINE_COLOR_READ_PIN, INPUT);
    pinMode(LINE_COLOR_SELECT_PIN, OUTPUT);
    digitalWrite(LINE_COLOR_SELECT_PIN, LOW);
    
    // Initialize color buffer with "other"
    for(int i = 0; i < COLOR_BUFFER_SIZE; i++) {
        colorBuffer[i] = 0;
    }
}

void SensorManager::update() {
    // Read sensors with error handling
    float rawLeftDist = readDistance(LEFT_SENSOR_PIN, lastValidLeftDistance); //!measured duration between 3000us and 10000us : conform with theoritical duration (100Hz update rate)
    float rawRightDist = readDistance(RIGHT_SENSOR_PIN, lastValidRightDistance); //!measured duration between 3000us and 10000us : conform with theoritical duration (100Hz update rate)
    
    // Apply constraints (0 to 1000mm)
    rawLeftDist = constrain(rawLeftDist, 0.0, 1000.0);
    rawRightDist = constrain(rawRightDist, 0.0, 1000.0);

    // Color check function and track counter
    checkTrackColor();
    trackCounter();

     // Update last valid distances
    lastValidLeftDistance = rawLeftDist;
    lastValidRightDistance = rawRightDist;

    globalState.leftDistance = static_cast<int16_t>(rawLeftDist);
    globalState.rightDistance = static_cast<int16_t>(rawRightDist);
    /*Serial.print(globalState.leftDistance);
    Serial.print(" - ");
    Serial.println(globalState.rightDistance);*/
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

void SensorManager::checkTrackColor() {
    // Read line sensor and update buffer
    int newColor = SensorManager::readLineColor(); //!measured duration between 150us and 200us (hidden from sunlight) : conform with theoritical duration (frequency around 20kHz)
    // Update buffer with new color, this buffer is used as a filter
    colorBuffer[colorBufferIndex] = newColor;
    colorBufferIndex = (colorBufferIndex + 1) % COLOR_BUFFER_SIZE;
    
    // Check if all colors in buffer are the same
    bool allSameColor = true;
    int firstColor = colorBuffer[0];
    for(int i = 1; i < COLOR_BUFFER_SIZE; i++) {
        if(colorBuffer[i] != firstColor) {
            allSameColor = false;
            break;
        }
    }
    
    // Update global state only if all colors are the same
    if(allSameColor) {
        globalState.trackColor = firstColor;
    }
}

int SensorManager::readLineColor() {
    int color = 0; // 0 for other, 1 for black, 2 for white, 3 for red, 4 for blue

    digitalWrite(LINE_COLOR_SELECT_PIN, LOW);
    int red = pulseIn(LINE_COLOR_READ_PIN, HIGH);
    digitalWrite(LINE_COLOR_SELECT_PIN, HIGH);
    int blue = pulseIn(LINE_COLOR_READ_PIN, HIGH);

    // examples of readings:
    //
    // Ground color | red | blue
    // -------------|-----|-----
    // Red          |  17 |  60
    // Red + sun    |  16 |  37
    // Gray         |  30 |  20
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

    //char buffer[30];
    //snprintf(buffer, sizeof(buffer), "Read red: %d, blue %d", red, blue);
    //Serial.println(buffer);

    // Logic based on the relationship between red and blue values
    if (red > 100 && blue > 100) {
        color = 1;
    }
    else if (red < 25 && blue < 25) {
        color = 2;
    }
    else if (red < blue - 15) {
        color = 3;
    }
    else if (blue < red - 15) {
        color = 4;
    }

    return color;
}

void SensorManager::trackCounter() {
    // Track is counted when we detect: blue line -> other -> red line -> other
    int currentColor = globalState.trackColor;
    //Serial.println(currentColor);
    // Detect start line (blue)
    if (currentColor != 4 && lastColor == 4) {
        hasPassedStartLine = true;
        Serial.println("Start line passed ");
    }
    
    // Detect finish line (red) after start line
    if (hasPassedStartLine && currentColor != 3 && lastColor == 3) {
         globalState.trackCount++;
        hasPassedStartLine = false;
        Serial.println("Finish line passed");
    }
    
    lastColor = currentColor;
}