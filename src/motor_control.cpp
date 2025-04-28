#include "motor_control.h"
#include "config.h"
#include <Arduino.h>

extern MotorData motorData;


MotorControl::MotorControl() {
    // Initialize pins
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_IN3, OUTPUT);
    pinMode(MOTOR2_IN4, OUTPUT);
    // Setup PWM channels for motors and servo
    setupPWM();
}

void MotorControl::setupPWM() {
    // Initialisation des broches pour MCPWM
    // Initialisation du enable PWM - Moteur 1
    mcpwm_gpio_init(MCPWM_UNIT, MCPWM0A, MOTOR1_ENA);

    // Initialisation du enable PWM - Moteur 2
    mcpwm_gpio_init(MCPWM_UNIT, MCPWM1A, MOTOR2_ENB);

    // Configuration des paramètres MCPWM
    mcpwm_config_t pwm_config;
    pwm_config.frequency = MOTOR_PWM_FREQ;     // Fréquence PWM définie dans config.h
    pwm_config.cmpr_a = 0;                     // Duty cycle initial à 0%
    pwm_config.cmpr_b = 0;                     // Non utilisé
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;  // Mode de duty cycle actif haut
    pwm_config.counter_mode = MCPWM_UP_COUNTER; // Compteur ascendant

    // Initialisation des timers MCPWM pour chaque moteur
    mcpwm_init(MCPWM_UNIT, MOTOR1_TIMER, &pwm_config);
    mcpwm_init(MCPWM_UNIT, MOTOR2_TIMER, &pwm_config);

    // Configuration du servo avec ESP32Servo
    // Force l'utilisation d'un timer spécifique (timer 3) pour éviter les conflits avec MCPWM
    ESP32PWM::allocateTimer(3);  // Alloue explicitement le timer 3 pour le servo

    steeringServo.setPeriodHertz(50); // 50 Hz standard pour les servos
    steeringServo.attach(SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);

    // Position initiale du servo à 35 degrés (centre)
    steeringServo.write(motorData.servoAngle);
}

void MotorControl::update() {
    if (!motorData.emergency) {
        // Contrôle des directions des moteurs via les pins IN1/IN2 et IN3/IN4
        // Motor 1 control
        digitalWrite(MOTOR1_IN1, (motorData.motor1Speed >= 0) == motorData.isForward ? HIGH : LOW);
        digitalWrite(MOTOR1_IN2, (motorData.motor1Speed >= 0) == motorData.isForward ? LOW : HIGH);

        // Motor 2 control
        digitalWrite(MOTOR2_IN3, (motorData.motor2Speed >= 0) == motorData.isForward ? HIGH : LOW);
        digitalWrite(MOTOR2_IN4, (motorData.motor2Speed >= 0) == motorData.isForward ? LOW : HIGH);

        // Contrôle PWM des moteurs avec MCPWM
        // Conversion de 0-255 à 0-100%
        float duty1 = abs(motorData.motor1Speed) / 255.0 * 100.0;
        float duty2 = abs(motorData.motor2Speed) / 255.0 * 100.0;

        // Application du PWM aux moteurs
        mcpwm_set_duty(MCPWM_UNIT, MOTOR1_TIMER, MCPWM_GEN_A, duty1);
        mcpwm_set_duty_type(MCPWM_UNIT, MOTOR1_TIMER, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);

        mcpwm_set_duty(MCPWM_UNIT, MOTOR2_TIMER, MCPWM_GEN_A, duty2);
        mcpwm_set_duty_type(MCPWM_UNIT, MOTOR2_TIMER, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);

        steeringServo.write(constrain(motorData.servoAngle, 20, 52));

    } else {
        // Emergency stop
        digitalWrite(MOTOR1_IN1, LOW);
        digitalWrite(MOTOR1_IN2, LOW);
        digitalWrite(MOTOR2_IN3, LOW);
        digitalWrite(MOTOR2_IN4, LOW);

        // Arrêt des moteurs avec MCPWM
        mcpwm_set_duty(MCPWM_UNIT, MOTOR1_TIMER, MCPWM_GEN_A, 0);
        mcpwm_set_duty_type(MCPWM_UNIT, MOTOR1_TIMER, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);

        mcpwm_set_duty(MCPWM_UNIT, MOTOR2_TIMER, MCPWM_GEN_A, 0);
        mcpwm_set_duty_type(MCPWM_UNIT, MOTOR2_TIMER, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
    }
}