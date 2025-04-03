#include "sensor_manager.h"
#include "config.h"
#include "state.h"
#include <Arduino.h>

// Variables pour le filtrage par moyenne glissante
#define NUM_READINGS 5
static int16_t leftReadings[NUM_READINGS] = {0};
static int16_t rightReadings[NUM_READINGS] = {0};
static uint8_t readIndex = 0;

void SensorManager::setup() {
    pinMode(LEFT_SENSOR_PIN, INPUT);
    pinMode(RIGHT_SENSOR_PIN, INPUT);
    pinMode(LINE_SENSOR_PIN, INPUT);
    
    // Initialisation des tableaux de lectures
    for (int i = 0; i < NUM_READINGS; i++) {
        leftReadings[i] = 450;  // Valeur initiale par défaut
        rightReadings[i] = 450; // Valeur initiale par défaut
    }
}

void SensorManager::update() {
    // Lecture réelle des capteurs au lieu des valeurs aléatoires
    int16_t rawLeftDist = readDistance(LEFT_SENSOR_PIN);
    int16_t rawRightDist = readDistance(RIGHT_SENSOR_PIN);
    
    // Si la lecture est valide (-1 indique une erreur)
    if (rawLeftDist >= 0) {
        leftReadings[readIndex] = rawLeftDist;
    }
    
    if (rawRightDist >= 0) {
        rightReadings[readIndex] = rawRightDist;
    }
    
    // Avancer l'index pour la prochaine lecture
    readIndex = (readIndex + 1) % NUM_READINGS;
    
    // Calculer la moyenne des lectures
    int16_t leftDist = calculateAverage(leftReadings, NUM_READINGS);
    int16_t rightDist = calculateAverage(rightReadings, NUM_READINGS);
    
    // Lecture du capteur de ligne
    bool line = digitalRead(LINE_SENSOR_PIN);
    
    // Mise à jour de l'état global
    globalState.leftDistance = leftDist;
    globalState.rightDistance = rightDist;
    globalState.lineDetected = line;
}

int16_t SensorManager::calculateAverage(int16_t* readings, uint8_t numReadings) {
    int32_t sum = 0;
    for (uint8_t i = 0; i < numReadings; i++) {
        sum += readings[i];
    }
    return sum / numReadings;
}

int16_t SensorManager::readDistance(uint8_t pin) {
    // La fonction pulseIn est bloquante, mais comme elle s'exécute sur un cœur dédié,
    // elle n'affectera pas la réactivité du système
    int16_t t = pulseIn(pin, HIGH, 30000); // Timeout de 30ms pour éviter de bloquer trop longtemps
    
    if (t == 0 || t > 1850) {
        // En cas d'erreur, retourner la dernière valeur valide
        return -1;  // Code d'erreur
    }
    
    // Convert pulse width to distance
    int16_t d = (t - 1000) * 2;
    return d < 0 ? 0 : d;
}