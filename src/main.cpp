#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "config.h"
#include "ble_manager.h"
#include "sensor_manager.h"
#include "motor_control.h"
#include "pid_controller.h"
#include "main.h"
#include "esp_task_wdt.h"

// Task handles
TaskHandle_t bleTaskHandle;

// Global instances
SensorManager sensorManager;
MotorControl motorControl;
PIDController pidController;

// Global data structures
SensorData sensorData;
MotorData motorData;
PIDData pidData;

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

    // Initialize sensor data
    sensorData.isAutoMode = false;
    sensorData.leftDistance = 0;
    sensorData.rightDistance = 0;
    sensorData.lineDetected = false;
    sensorData.autoModeMotorSpeed = 0;

    // Initialize motor data
    motorData.emergency = true;
    motorData.motor1Speed = 0;
    motorData.motor2Speed = 0;
    motorData.servoAngle = 35;
    motorData.isForward = true;  // Initialize to forward direction

    // Create tasks
    xTaskCreatePinnedToCore(bleTask, "BLE", 4096, NULL, 1, &bleTaskHandle, 0); // Tâche émission BLE sur Core 0 avec priorité 1

    Serial.println("Setup complete - All tasks created");
}

void loop() {
    sensorManager.update();
    pidController.update(pidData);
    motorControl.update();
    delay(PILOT_INTERVAL);
}
