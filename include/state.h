#include <stdint.h>
#include <Arduino.h>

#pragma once

struct GlobalState {
    volatile int16_t leftDistance;
    volatile int16_t rightDistance;
    volatile bool changeSpeed;
    volatile bool hasPassedStartLine = false;
    volatile bool lineDetected;
    volatile bool isAutoMode;
    volatile int motor1Speed;
    volatile int motor2Speed;
    volatile int autoModeMotorSpeed;
    volatile int servoAngle;
    volatile bool emergency;
    volatile bool isForward;  // true for forward, false for backward
    volatile int trackCount;
    volatile int trackColor;  // 0 for other, 1 for black, 2 for white, 3 for red, 4 for blue
};

extern GlobalState globalState; 