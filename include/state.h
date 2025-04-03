#include <stdint.h>

#pragma once

struct GlobalState {
    volatile int16_t leftDistance;
    volatile int16_t rightDistance;
    volatile bool lineDetected;
    volatile bool isAutoMode;
    volatile int motor1Speed;
    volatile int motor2Speed;
    volatile int servoAngle;
    volatile bool emergency;
    volatile bool isForward;  // true for forward, false for backward
};

extern GlobalState globalState; 