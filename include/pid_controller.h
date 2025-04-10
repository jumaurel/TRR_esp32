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
    static void setServoRange(int value);  // Range of servo movement in degrees
    
private:
    static float Kp;
    static float Ki;
    static float Kd;
    static float targetDistance;
    static float lastError;
    static float integral;
    static int baseSpeed;
    static int servoRange;  // Range of servo movement in degrees
    static int servoCenter; // Center position of servo in degrees
}; 