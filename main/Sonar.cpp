#include "Sonar.hpp"

#include "EspWait.hpp"

#include <esp_log.h>

#include <optional>

static constexpr auto TAG = "Sonar";

Sonar::Sonar(int triggerPin, int echoPin) :
    mTriggerPin(triggerPin)
{
    gpio_install_isr_service(0);
    {
        gpio_config_t io_conf{};
        io_conf.intr_type = GPIO_INTR_ANYEDGE;
        io_conf.pin_bit_mask = 1ull << echoPin;
        io_conf.mode = GPIO_MODE_INPUT;
        gpio_config(&io_conf);

        gpio_isr_handler_add((gpio_num_t) echoPin, &Sonar::handleInterrupt, this);
    }
    {
        gpio_config_t io_conf{};
        io_conf.pin_bit_mask = 1ull << triggerPin;
        io_conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&io_conf);
    }
}
int asd = 2;
void Sonar::startMeasurement()
{
    if (asd == 10) {
        ESP_LOGI(TAG, "Ping %f", delay() ? *delay() : 0.0f);
        asd = 0;
        gpio_set_level((gpio_num_t) mTriggerPin, 1);
        EspWait::microseconds(10);
        gpio_set_level((gpio_num_t) mTriggerPin, 0);
    }
    ++asd;
}

void IRAM_ATTR Sonar::handleInterrupt(void* arg)
{
    auto now = esp_timer_get_time();
    ets_printf("a");
    auto self = static_cast<Sonar *>(arg);
    if (self->mMeasurementBegin) {
        self->mDelay = now - self->mMeasurementBegin;
        self->mMeasurementBegin = 0;
    } else {
        self->mMeasurementBegin = now;
    }
}

