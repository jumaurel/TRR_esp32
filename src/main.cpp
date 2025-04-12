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

SemaphoreHandle_t globalMutex;

// Task handles
TaskHandle_t bleTaskHandle;
TaskHandle_t sensorTaskHandle;
TaskHandle_t controlTaskHandle;
TaskHandle_t pidTaskHandle;

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

void sensorTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (true) {
        if (xSemaphoreTake(globalMutex, portMAX_DELAY)) {
            SensorManager::update();
            xSemaphoreGive(globalMutex);
        } 
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_INTERVAL));
    }
}

void controlTask(void* parameter) {
    while (true) {       
        if (xSemaphoreTake(globalMutex, portMAX_DELAY)) {
            MotorControl::update();
            xSemaphoreGive(globalMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz control loop
    }
}

void pidTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (true) {     
        if (xSemaphoreTake(globalMutex, portMAX_DELAY)) {
            PIDController::update();
            xSemaphoreGive(globalMutex);
        }
        
        // Ensure we yield to other tasks
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PID_INTERVAL));
    }
}

void setup() {
    Serial.begin(115200);
    
    // Initialize mutex
    globalMutex = xSemaphoreCreateMutex();

        // Initialize global state
    globalState.isAutoMode = false;
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

    // Create tasks
    //xTaskCreatePinnedToCore(bleTask, "BLE", 4096, NULL, 1, &bleTaskHandle, 0); // Tâche émission BLE sur Core 0 avec priorité 1
    xTaskCreatePinnedToCore(sensorTask, "Sensor", 2048, NULL, 2, &sensorTaskHandle, 0);  // Tâche capteurs sur Core 0 avec priorité 2
    xTaskCreatePinnedToCore(controlTask, "Control", 4096, NULL, 1, &controlTaskHandle, 1); // Tâche contrôle moteurs sur Core 1 avec priorité 1
    xTaskCreatePinnedToCore(pidTask, "PID", 2048, NULL, 1, &pidTaskHandle, 1); // Tâche PID sur Core 1 avec priorité 1
    
    Serial.println("Setup complete - All tasks created");
}

void loop() {
    delay(100);  // Small delay to prevent watchdog reset
}
