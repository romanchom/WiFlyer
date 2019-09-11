#pragma once

#include <esp_timer.h>
#include <driver/gpio.h>

#include <optional>
#include <cstdint>

struct Sonar
{
    explicit Sonar(int triggerPin, int echoPin);

    std::optional<float> delay()
    {
        if (mDelay) {
           return float(mDelay) / 1000000.0f;
        } else {
            return {};
        }
    }
    
    void startMeasurement();

private:
    static void IRAM_ATTR handleInterrupt(void* arg);
    int mTriggerPin;
    int64_t mMeasurementBegin;
    int32_t mDelay;
};

