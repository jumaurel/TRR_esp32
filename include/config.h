#pragma once

// Pin Definitions
#define LEFT_SENSOR_PIN  35    // Capteur gauche
#define RIGHT_SENSOR_PIN 34    // Capteur droit
#define LINE_SENSOR_PIN  4    // Capteur ligne d'arrivée

#define MOTOR1_IN1  26   // Motor 1 control pin 1
#define MOTOR1_IN2  25   // Motor 1 control pin 2
#define MOTOR1_ENA  14   // Motor 1 enable pin (PWM)

#define MOTOR2_IN3  26   // Motor 2 control pin 1
#define MOTOR2_IN4  25   // Motor 2 control pin 2
#define MOTOR2_ENB  13   // Motor 2 enable pin (PWM)

#define SERVO_PIN   16   // Servo control pin

// Bluetooth UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CONTROL_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SENSOR_CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// Constants
#define MOTOR_PWM_FREQ     5000
#define MOTOR_PWM_RES      8       // 8-bit resolution (0-255)
#define PID_INTERVAL       100     // PID calculation interval in ms
#define SENSOR_INTERVAL    100     // Sensor reading interval in ms
#define BLE_NOTIFY_INTERVAL 100    // BLE notification interval in ms 