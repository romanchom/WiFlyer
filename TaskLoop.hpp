#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct TaskLoop
{
    TaskLoop(int frequency)
    {
        mNextStepTime = xTaskGetTickCount();
        mTicks = 1000 / (portTICK_PERIOD_MS * frequency);
    }

    void wait()
    {
        vTaskDelayUntil(&mNextStepTime, mTicks);
    }
private:
    TickType_t mNextStepTime;
    TickType_t mTicks;
};
