#pragma once

#include <ESP32Servo.h>

class MotorControl {
public:
    static void setup();
    static void update();
    
private:
    static Servo steeringServo;
    static void setupPWM();
}; 