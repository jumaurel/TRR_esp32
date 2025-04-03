#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include "driver/mcpwm.h"

class MotorControl {
public:
    static void setup();
    static void update();
    
private:
    // Constantes pour le contrôle du servo
    static const uint16_t SERVO_MIN_PULSE = 500;  // 500 µs (0 degrés)
    static const uint16_t SERVO_MAX_PULSE = 2400; // 2400 µs (180 degrés)
    
    // Constantes pour MCPWM
    static const mcpwm_unit_t MCPWM_UNIT = MCPWM_UNIT_0;
    static const mcpwm_timer_t MOTOR1_TIMER = MCPWM_TIMER_0;
    static const mcpwm_timer_t MOTOR2_TIMER = MCPWM_TIMER_1;
    
    static void setupPWM();
    static Servo steeringServo; // Instance de Servo pour piloter le servo
}; 