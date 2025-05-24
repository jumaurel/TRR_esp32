#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "config.h"
#include "state.h"
#include "ble_manager.h"
#include "sensor_manager.h"
#include "motor_control.h"
#include "pid_controller.h"
#include "main.h"
#include "esp_task_wdt.h"

// Global variables
bool switchActivated = false;
unsigned long switchActivationTime = 0;
unsigned long emergencyStartTime = 0;  // Timer pour l'état d'urgence
bool emergencyTimerActive = false;     // Indique si le timer est actif

// Variables for light detection
const int LIGHT_SAMPLE_SIZE = 10;  // Taille de la moyenne glissante
int lightSamples[LIGHT_SAMPLE_SIZE];  // Tableau pour stocker les échantillons
int lightSampleIndex = 0;  // Index actuel dans le tableau
int lightAverage = 0;  // Moyenne actuelle
bool isLightDetectionActive = false;  // État de la détection de lumière

// Task handles
TaskHandle_t bleTaskHandle;

// Interrupt Service Routine for the switch button
void IRAM_ATTR switchISR() {
    switchActivationTime = millis();
}

// Tasks
void bleTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        if (BLEManager::isConnected()) {
            BLEManager::sendSensorData();
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(BLE_NOTIFY_INTERVAL));
    }
}

void setup() {
    Serial.begin(115200);


    // Initialize global state
    globalState.trackCount = 0;
    globalState.isAutoMode = true;
    globalState.emergency = true;
    globalState.motor1Speed = 0;
    globalState.motor2Speed = 0;
    globalState.servoAngle = 35;
    globalState.isForward = true;  // Initialize to forward direction

    // Initialize components
    BLEManager::setup();
    MotorControl::setup();
    PIDController::setup();
    SensorManager::setup();

    // Setup switch button interrupt
    pinMode(SWITCH_BUTTON, INPUT);
    attachInterrupt(digitalPinToInterrupt(SWITCH_BUTTON), switchISR, RISING);

    // Create tasks
    //xTaskCreatePinnedToCore(bleTask, "BLE", 4096, NULL, 1, &bleTaskHandle, 0); // Tâche émission BLE sur Core 0 avec priorité 1

    Serial.println("Setup complete - All tasks created");

    // wait 5 seconds before starting the car
    delay(5000);
    globalState.emergency = false;
}


void loop() {
    //checkLightDetection();  // Seule fonction de contrôle de démarrage
    
    //START SIGNAL DETECTOR
    /*Serial.println(analogRead(RED_SPOT_PIN));
    if(analogRead(RED_SPOT_PIN) < 3000){
        globalState.emergency = false;
    }*/

    // Gestion du timer d'urgence
    if (!globalState.emergency && !emergencyTimerActive) {
        emergencyStartTime = millis();
        emergencyTimerActive = true;
    }
    
    if (emergencyTimerActive && (millis() - emergencyStartTime >= 75000)) {  // 1mn20 = 80000ms
        globalState.emergency = true;
        emergencyTimerActive = false;
    }

    // Start the car if the switch is activated
    //if (switchActivated) {
        SensorManager::update(); //? measured function duration : between 9700 and 20500us
        PIDController::update(); //? measured function duration : ~10000us (between 9700 and 10500us)     
        MotorControl::update(); //? measured function duration : 18us
         
    //} 

    // Stop the car if the track count is 6 => this has been removed because it was causing the car to stop prematurely when line detection was not working => stop is now handled by the emergency timer after 1mn20
    /*if(globalState.trackCount == 6){
        globalState.emergency = true;
        globalState.trackCount = 0;
    }*/

    // Going faster during straight line
    if(globalState.hasPassedStartLine == true){
        globalState.motor1Speed = 50 * 255 / 100;
        globalState.motor2Speed = 50 * 255 / 100;
     }
    else{
        globalState.motor1Speed = 40 * 255 / 100;
        globalState.motor2Speed = 40 * 255 / 100;
    }

    //delay(PILOT_INTERVAL);
}


void checkSwitch() {
    // Check if switch is HIGH, then wait for 3 seconds before activating the car
    if (digitalRead(SWITCH_BUTTON) == HIGH && 
        (millis() - switchActivationTime >= STARTUP_DELAY)) {
        switchActivated = true;
        globalState.emergency = false;
    }
    else {  
        switchActivated = false;
        globalState.emergency = true;
        digitalWrite(MOTOR1_IN1, LOW);
        digitalWrite(MOTOR1_IN2, LOW);
        digitalWrite(MOTOR2_IN3, LOW);
        digitalWrite(MOTOR2_IN4, LOW);
    }
}

void checkLightDetection() {
    if (!digitalRead(SWITCH_BUTTON)) {
        // Réinitialiser si l'interrupteur est désactivé
        isLightDetectionActive = false;
        lightSampleIndex = 0;
        lightAverage = 0;
        return;
    }

    // Activer la détection si l'interrupteur est activé
    if (!isLightDetectionActive) {
        isLightDetectionActive = true;
        // Initialiser le tableau avec la première valeur
        int initialValue = analogRead(RED_SPOT_PIN);
        for (int i = 0; i < LIGHT_SAMPLE_SIZE; i++) {
            lightSamples[i] = initialValue;
        }
        lightAverage = initialValue;
        return;
    }

    // Lire la nouvelle valeur
    int currentLight = analogRead(RED_SPOT_PIN);
    
    // Mettre à jour la moyenne glissante
    lightAverage = lightAverage - (lightSamples[lightSampleIndex] / LIGHT_SAMPLE_SIZE);
    lightSamples[lightSampleIndex] = currentLight;
    lightAverage = lightAverage + (currentLight / LIGHT_SAMPLE_SIZE);
    
    // Mettre à jour l'index
    lightSampleIndex = (lightSampleIndex + 1) % LIGHT_SAMPLE_SIZE;

    // Vérifier si le changement est significatif
    if (abs(currentLight - lightAverage) > 100) {
        globalState.emergency = false;
        isLightDetectionActive = false;
    }
}