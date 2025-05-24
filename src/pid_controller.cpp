#include "pid_controller.h"
#include "config.h"
#include "state.h"
#include <Arduino.h>

// Static member initialization
float PIDController::Kp = 0.025;
float PIDController::Ki = 0.0;
float PIDController::Kd = 0.004;
float PIDController::targetDistance = 500; // 500mm from wall
float PIDController::lastError = 0;
float PIDController::integral = 0;
int PIDController::servoCenter = 35; // Default center position of servo
unsigned long PIDController::loopDurationStart = 0;
unsigned long PIDController::loopDuration = 0;

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

void PIDController::update() {
    loopDuration = millis() - loopDurationStart;
    /*if (!globalState.isAutoMode){ // si mode manuel, on ne fait pas le calcul du PID
        delay(10);
        return;
    }*/

    // Calculate error (average of left and right distances)
    float error = globalState.leftDistance - globalState.rightDistance;

    // PID calculation
    integral += error * (loopDuration / 1000.0);
    float derivative = (error - lastError) / (loopDuration / 1000.0);
    float output = Kp * error + Ki * integral + Kd * derivative;

    // Convert PID output to servo angle (0-180 degrees)
    int servoAngle = constrain(servoCenter + output, 20, 52);
    globalState.servoAngle = servoAngle;

    lastError = error;

    loopDurationStart = millis();
}