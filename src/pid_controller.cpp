#include "pid_controller.h"
#include "config.h"
#include "state.h"
#include <Arduino.h>

// Static member initialization
float PIDController::Kp = 1.0;
float PIDController::Ki = 0.0;
float PIDController::Kd = 0.0;
float PIDController::targetDistance = 500; // 500mm from wall
float PIDController::lastError = 0;
float PIDController::integral = 0;
int PIDController::baseSpeed = 128; // Default 50% speed

void PIDController::setup() {
    reset();
}

void PIDController::reset() {
    lastError = 0;
    integral = 0;
}

// Setter methods implementation
void PIDController::setKp(float value) {
    Kp = value;
}

void PIDController::setKi(float value) {
    Ki = value;
}

void PIDController::setKd(float value) {
    Kd = value;
}

void PIDController::setBaseSpeed(int value) {
    baseSpeed = constrain(value, 0, 255);  // Ensure value is between 0 and 255
}

void PIDController::update() {
    if (!globalState.isAutoMode || globalState.emergency) return;
    
    // Calculate error (average of left and right distances)
    float currentDistance = (globalState.leftDistance + globalState.rightDistance) / 2.0;
    float error = targetDistance - currentDistance;
    
    // PID calculation
    integral += error * (PID_INTERVAL / 1000.0);
    float derivative = (error - lastError) / (PID_INTERVAL / 1000.0);
    float output = Kp * error + Ki * integral + Kd * derivative;
    
    // Apply PID output to motors using the programmable baseSpeed
    globalState.motor1Speed = constrain(baseSpeed + output, -255, 255);
    globalState.motor2Speed = constrain(baseSpeed - output, -255, 255);
    
    lastError = error;
} 