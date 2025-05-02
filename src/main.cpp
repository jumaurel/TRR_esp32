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
unsigned long functionDuration = 0;
unsigned long sensorMeasurementStart = 0;
unsigned long sensorMeasurementDuration = 0;

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
}

void loop() {
    // Check switch before starting the car
    //checkSwitch();

    // Start the car if the switch is activated
    //if (switchActivated) {
        sensorMeasurementStart = millis();
        SensorManager::update(); //? measured function duration : between 9700 and 20500us
        sensorMeasurementDuration = millis() - sensorMeasurementStart;
        PIDController::update(sensorMeasurementDuration); //? measured function duration : ~10000us (between 9700 and 10500us)     
        MotorControl::update(); //? measured function duration : 18us
    //} 

    if(globalState.trackCount == 2){
        globalState.emergency = true;
        globalState.trackCount = 0;
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