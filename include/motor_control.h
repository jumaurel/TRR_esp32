#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include "driver/mcpwm.h"

#define SERVO_CENTER 35

struct MotorData {
    int motor1Speed;
    int motor2Speed;
    int servoAngle;
    bool isForward;  // true for forward, false for backward
    bool emergency;
};

class MotorControl {
public:
    // Constructor replaces setup
    MotorControl();

    void update(MotorData &motorDdata);

private:
    // Constants for servo control
    static const uint16_t SERVO_MIN_PULSE = 500;  // 500 µs (0 degrees)
    static const uint16_t SERVO_MAX_PULSE = 2400; // 2400 µs (180 degrees)

    // Constants for MCPWM
    static const mcpwm_unit_t MCPWM_UNIT = MCPWM_UNIT_0;
    static const mcpwm_timer_t MOTOR1_TIMER = MCPWM_TIMER_0;
    static const mcpwm_timer_t MOTOR2_TIMER = MCPWM_TIMER_1;

    void setupPWM();
    Servo steeringServo; // Instance of Servo to control the servo
};