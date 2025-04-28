#include "pid_controller.h"
#include "config.h"
#include "state.h"
#include <Arduino.h>

// Static member initialization
float PIDController::Kp = 0.1;
float PIDController::Ki = 0.0;
float PIDController::Kd = 0.0;
float PIDController::targetDistance = 500; // 500mm from wall
float PIDController::lastError = 0;
float PIDController::integral = 0;
int PIDController::servoCenter = 35; // Default center position of servo
int PIDController::autoModeMotorSpeed = 0; // Default to 0% power

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

void PIDController::setAutoModeMotorSpeed(int value) {
    autoModeMotorSpeed = value;
}

void PIDController::update() {
    if (!globalState.isAutoMode){
        delay(10);
        return;
    }

    // Calculate error (average of left and right distances)
    float error = globalState.leftDistance - globalState.rightDistance;

    // PID calculation
    integral += error * (PILOT_INTERVAL / 1000.0);
    float derivative = (error - lastError) / (PILOT_INTERVAL / 1000.0);
    float output = Kp * error + Ki * integral + Kd * derivative;

    //Serial.println(error);
    /*Serial.print(output);
    Serial.print(" - ");
    Serial.println(constrain(servoCenter + output, 20, 52));
    */
    // Apply base speed to both motors

    globalState.motor1Speed = autoModeMotorSpeed;
    globalState.motor2Speed = autoModeMotorSpeed;

    // Convert PID output to servo angle (0-180 degrees)
    int servoAngle = constrain(servoCenter + output, 20, 52);
    globalState.servoAngle = servoAngle;

    lastError = error;
}