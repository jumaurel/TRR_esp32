#pragma once

class PIDController {
public:
    static void setup();
    static void update();
    static void reset();
    
    // Add setter methods for PID parameters
    static void setKp(float value);
    static void setKi(float value);
    static void setKd(float value);
    static void setBaseSpeed(int value);
    
private:
    static float Kp;
    static float Ki;
    static float Kd;
    static float targetDistance;
    static float lastError;
    static float integral;
    static int baseSpeed;
}; 