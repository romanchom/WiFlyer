#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct TaskNotification
{
    void waitForEvent(uint32_t bits, TickType_t timeout = portMAX_DELAY)
    {
        while ((mNotificationValue & bits) != bits) {
            xTaskNotifyWait(0, bits, &mNotificationValue, portMAX_DELAY);
        }
        mNotificationValue &= ~bits;
    }
private:
    uint32_t mNotificationValue = 0;
};
