#include "pid_controller.h"
#include "config.h"
#include "state.h"
#include <Arduino.h>

// Static member initialization

// Constructor implementation
PIDController::PIDController() : Kp(0.1), Ki(0.0), Kd(0.08), integral(0), lastError(0), autoModeMotorSpeed(0) {
    // Initialization logic for PID constants and instance variables
}

void PIDController::reset() {
    lastError = 0;
    integral = 0;
}

// Getter methods implementation
float PIDController::getKp() const {
    return Kp;
}

float PIDController::getKi() const {
    return Ki;
}

float PIDController::getKd() const {
    return Kd;
}

float PIDController::getIntegral() const {
    return integral;
}

float PIDController::getLastError() const {
    return lastError;
}

int PIDController::getAutoModeMotorSpeed() const {
    return autoModeMotorSpeed;
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

void PIDController::setIntegral(float value) {
    integral = value;
}

void PIDController::setLastError(float value) {
    lastError = value;
}

void PIDController::setAutoModeMotorSpeed(int value) {
    autoModeMotorSpeed = value;
}

void PIDController::update() {
    if (!globalState.isAutoMode) {
        delay(10);
        return;
    }

    // Calculate error (average of left and right distances)
    float error = globalState.leftDistance - globalState.rightDistance;

    // PID calculation
    integral += error * (PILOT_INTERVAL / 1000.0);
    float derivative = (error - lastError) / (PILOT_INTERVAL / 1000.0);
    float output = Kp * error + Ki * integral + Kd * derivative;

    // Apply base speed to both motors
    globalState.motor1Speed = autoModeMotorSpeed;
    globalState.motor2Speed = autoModeMotorSpeed;

    // Convert PID output to servo angle (0-180 degrees)
    int servoAngle = constrain(servoCenter + output, 20, 52);
    globalState.servoAngle = servoAngle;

    lastError = error;
}