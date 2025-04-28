#pragma once

#include <Arduino.h>
#include "config.h"

class PIDController {
public:
    // Constructor replaces setup
    PIDController();

    void update();
    void reset();

    // Getters and setters for PID constants
    float getKp() const;
    void setKp(float kp);

    float getKi() const;
    void setKi(float ki);

    float getKd() const;
    void setKd(float kd);

    // Getters and setters for instance variables
    float getIntegral() const;
    void setIntegral(float value);

    float getLastError() const;
    void setLastError(float value);

    int getAutoModeMotorSpeed() const;
    void setAutoModeMotorSpeed(int value);

private:
    // PID constants
    float Kp;
    float Ki;
    float Kd;

    // Instance variables
    float integral;
    float lastError;
    int autoModeMotorSpeed;

    // Servo center position
    static const int servoCenter = 35; // Default center position of servo
};