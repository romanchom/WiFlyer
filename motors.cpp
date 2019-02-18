#include "PWM.hpp"

static void testMotorsTask(void *arg)
{
    auto timer = PWMTimer(0, 100, 13);
    auto pwm0 = PWM(&timer, GPIO_NUM_23, 0.0f);
    auto pwm1 = PWM(&timer, GPIO_NUM_4, 0.0f);
    auto pwm2 = PWM(&timer, GPIO_NUM_12, 0.0f);
    auto pwm3 = PWM(&timer, GPIO_NUM_33, 0.0f);

    constexpr float period = 1.0f / 100.0f;
    constexpr float zero = 0.001f;
    constexpr float one = 0.002f;

    float percentage = 0.0f;

    while (true) {
        percentage += 0.01f;
        if (percentage >= 0.4f) {
            percentage = 0.0f;
        }

        float duty = (zero + (one - zero) * percentage) / period;

        pwm0.duty(duty);
        pwm1.duty(duty);
        pwm2.duty(duty);
        pwm3.duty(duty);

        ESP_LOGI(TAG, "%f%% %f", percentage, duty);
        if (percentage == 0.0f) {
            // vTaskDelay(1000);
        }
        vTaskDelay(10);
    }
}
