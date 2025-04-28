#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include "driver/mcpwm.h"

class MotorControl {
public:
    // Constructor replaces setup
    MotorControl();

    void update();

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