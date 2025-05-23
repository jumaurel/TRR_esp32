#pragma once

// Pin Definitions
#define LEFT_SENSOR_PIN  19    // Capteur gauche
#define RIGHT_SENSOR_PIN 18    // Capteur droit
#define FRONT_SENSOR_PIN 4

#define MOTOR1_IN1  26   // Motor 1 control pin 1
#define MOTOR1_IN2  25   // Motor 1 control pin 2
#define MOTOR1_ENA  14   // Motor 1 enable pin (PWM)

#define MOTOR2_IN3  26   // Motor 2 control pin 1
#define MOTOR2_IN4  25   // Motor 2 control pin 2
#define MOTOR2_ENB  13   // Motor 2 enable pin (PWM)

#define SERVO_PIN   16   // Servo control pin
#define LINE_COLOR_READ_PIN 23 // detecteur de ligne de couleur
#define LINE_COLOR_SELECT_PIN 5

// Bluetooth UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CONTROL_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SENSOR_CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// Constants
#define MOTOR_PWM_FREQ     5000
#define MOTOR_PWM_RES      8       // 8-bit resolution (0-255)
#define PILOT_INTERVAL     20
#define BLE_NOTIFY_INTERVAL 100    // BLE notification interval in ms

#define SWITCH_BUTTON 21
#define STARTUP_DELAY 3000  // 3 second delay after switch button is pressed

#define RED_SPOT_PIN 34    // Pin pour le capteur de spot rouge de départ
