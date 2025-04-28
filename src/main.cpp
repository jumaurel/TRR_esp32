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

// Task handles
TaskHandle_t bleTaskHandle;

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

    Serial.println("Setup complete - All tasks created");
}

void loop() {
    SensorManager::update();
    PIDController::update();
    MotorControl::update();
    delay(PILOT_INTERVAL);
}
