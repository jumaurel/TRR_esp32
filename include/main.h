// main.h
#pragma once

#include <cstddef>  // For size_t
#include <cstdint>  // For uint8_t

// Mutex for protecting global state
extern SemaphoreHandle_t globalMutex;

// Task handles
extern TaskHandle_t bleTaskHandle;
extern TaskHandle_t sensorTaskHandle;
extern TaskHandle_t controlTaskHandle;
extern TaskHandle_t pidTaskHandle;