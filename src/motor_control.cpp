#include "motor_control.h"
#include "config.h"
#include "state.h"
#include <Arduino.h>

Servo MotorControl::steeringServo;

void MotorControl::setup() {
    // Initialize pins
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_IN3, OUTPUT);
    pinMode(MOTOR2_IN4, OUTPUT);
    
    // Setup PWM channels for motors
    setupPWM();
    
    // Initialize servo
    ESP32PWM::allocateTimer(0);
    steeringServo.setPeriodHertz(50);
    steeringServo.attach(SERVO_PIN, 500, 2400);
}

void MotorControl::setupPWM() {
   // Initialize LEDC channels first
    ledcSetup(0, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcSetup(1, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    
    // Then attach pins to channels
    ledcAttachPin(MOTOR1_ENA, 0);
    ledcAttachPin(MOTOR2_ENB, 1);
}

void MotorControl::update() {
    if (!globalState.emergency) {
        // Motor 1 control
        digitalWrite(MOTOR1_IN1, globalState.motor1Speed >= 0 ? HIGH : LOW);
        digitalWrite(MOTOR1_IN2, globalState.motor1Speed >= 0 ? LOW : HIGH);
        ledcWrite(0, abs(globalState.motor1Speed));
        
        // Motor 2 control
        digitalWrite(MOTOR2_IN3, globalState.motor2Speed >= 0 ? HIGH : LOW);
        digitalWrite(MOTOR2_IN4, globalState.motor2Speed >= 0 ? LOW : HIGH);
        ledcWrite(1, abs(globalState.motor2Speed));
        
        // Servo control
        steeringServo.write(globalState.servoAngle + 90); // Convert -90/90 to 0/180
    } else {
        // Emergency stop
        digitalWrite(MOTOR1_IN1, LOW);
        digitalWrite(MOTOR1_IN2, LOW);
        digitalWrite(MOTOR2_IN3, LOW);
        digitalWrite(MOTOR2_IN4, LOW);
        ledcWrite(0, 0);
        ledcWrite(1, 0);
    }
} 