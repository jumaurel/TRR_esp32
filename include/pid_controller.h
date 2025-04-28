#pragma once

#include <Arduino.h>
#include "config.h"

struct PIDData {
    float P;
    float I;
    float D;
    float integral;
    float derivative;
    float output;
    float error;
    float lastError;
};

class PIDController {
public:
    // Constructor replaces setup
    PIDController();

    void reset(PIDData &pidData);
    void update(PIDData &pidData);
};