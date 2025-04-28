#include "pid_controller.h"
#include "config.h"
#include "sensor_manager.h"
#include "motor_control.h"
#include <Arduino.h>

PIDController::PIDController() {
}

void PIDController::reset(PIDData &pidData) {
    pidData.lastError = 0;
    pidData.integral = 0;
}

void PIDController::update(SensorData &sensorData, PIDData &pidData, MotorData &motorData) {
    if (!sensorData.isAutoMode) {
        delay(10);
        return;
    }

    // Calculate error (average of left and right distances)
    pidData.error = sensorData.leftDistance - sensorData.rightDistance;

    // PID calculation
    pidData.integral += pidData.error * (PILOT_INTERVAL / 1000.0);
    pidData.derivative = (pidData.error - pidData.lastError) / (PILOT_INTERVAL / 1000.0);
    pidData.output = pidData.P * pidData.error + pidData.I * pidData.integral + pidData.D * pidData.derivative;

    // Apply base speed to both motors
    motorData.motor1Speed = sensorData.autoModeMotorSpeed;
    motorData.motor2Speed = sensorData.autoModeMotorSpeed;

    // Convert PID output to servo angle (0-180 degrees)
    int servoAngle = constrain(SERVO_CENTER + pidData.output, 20, 52);
    motorData.servoAngle = servoAngle;

    pidData.lastError = pidData.error;
}