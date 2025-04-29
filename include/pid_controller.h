#pragma once

#include <Arduino.h>
#include "config.h"
#include "sensor_manager.h"
#include "motor_control.h"

// Size of the window over which we compute the moving average error
#define MOVING_AVERAGE_SIZE 5

struct PIDData {
    float P;
    float I;
    float D;
    float integral;
    float derivative;
    float output;
    float error;
    float lastError;
    float averageError;
};

class PIDController {
public:
    // Constructor replaces setup
    PIDController();

    void reset(PIDData &pidData);
    void update(SensorData &sensorData, PIDData &pidData, MotorData &motorData);

private:
    int maxSpeed(float output);
    int lastErrorIndex;
    float lastErrors[MOVING_AVERAGE_SIZE];
    void addToMovingAverage(PIDData &pidData, float error);
};