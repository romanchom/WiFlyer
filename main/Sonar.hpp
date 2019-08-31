#pragma once

#include "EspWait.hpp"

#include <driver/gpio.h>
#include <esp_log.h>

#include <optional>

struct Sonar
{
    explicit Sonar(int triggerPin, int echoPin);

    std::optional<float> distance();
    void startMeasurement();

private:
    static void IRAM_ATTR handleInterrupt(void* arg);
    int mTriggerPin;
    std::optional<float> mDistance;
};

