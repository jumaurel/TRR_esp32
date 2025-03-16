#pragma once

class PIDController {
public:
    static void setup();
    static void update();
    static void reset();
    
private:
    static float Kp;
    static float Ki;
    static float Kd;
    static float targetDistance;
    static float lastError;
    static float integral;
}; 