#include "pid_controller.h"
#include "config.h"
#include "sensor_manager.h"
#include "motor_control.h"
#include <Arduino.h>

PIDController::PIDController() : lastErrors{0}, lastErrorIndex(0) {
}

void PIDController::reset(PIDData &pidData) {
    std::fill(std::begin(lastErrors), std::end(lastErrors), 0.0f);
    lastErrorIndex = 0;
    pidData.integral = 0;
    pidData.lastError = 0;
    pidData.averageError = 0;
}

void PIDController::update(SensorData &sensorData, PIDData &pidData, MotorData &motorData) {
    if (!sensorData.isAutoMode) {
        delay(10);
        return;
    }

    // Calculate error (average of left and right distances)
    pidData.error = sensorData.leftDistance - sensorData.rightDistance;
    addToMovingAverage(pidData, pidData.error); // Sets pidData.averageError correctly

    // TODO: when turning, we should try to cut inside turns: when turning left,
    // to aim closer to the left side, and when turning right, closer to the right side.
    //
    // This requires to determine whether we're turning.
    //
    // More precisely, whether we're turning as a micro-correction, or in order to
    // follow an actual turn on the track.
    //
    // servoAngle therefore isn't an adequate input, it's likely to runaway because of
    // micro corrections: a slight left turn would make us want to go and stick
    // to the left side, which would amplify the leftward servo angle, etc.
    //
    // We should check whether the moving average fares better.

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

    int motorSpeed = maxSpeed(pidData.output);
    // motorData.motor1Speed = motorData.motor2Speed = motorSpeed;

    pidData.lastError = pidData.error;
}

void PIDController::addToMovingAverage(PIDData &pidData, float error) {
    lastErrorIndex++;
    if(lastErrorIndex == MOVING_AVERAGE_SIZE) {
        lastErrorIndex = 0;
    }
    lastErrors[lastErrorIndex] = error;
    float sumOfErrors = 0;
    for(int i=0; i<MOVING_AVERAGE_SIZE; i++) {
        sumOfErrors += lastErrors[i];
    }
    pidData.averageError = sumOfErrors / MOVING_AVERAGE_SIZE;
}

// TODO: Compute the maximal reasonable speed for a given output/angle.
// The point is to slow down in sharper corners, and go faster in straight line and wider turns.
// We may also try to differentiate left/right speeds, to add some rear wheel differential steering
// to the front-wheel steering in sharper turns.
// Probably base it on moving window average error, to avoid jerks on each micro correction.
//
int PIDController::maxSpeed(float output) {
    return 0;
}