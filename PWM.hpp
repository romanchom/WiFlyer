#pragma once

#include "driver/ledc.h"

struct PWMTimer
{
    PWMTimer(uint32_t timerId, uint32_t frequency, uint32_t resolutionBits) :
        mTimerId((ledc_timer_t) timerId),
        mCurrentChannel(0),
        mSteps(1 << resolutionBits),
        mFrequency(frequency)
    {
        ledc_timer_config_t ledc_timer;

        ledc_timer.duty_resolution = (ledc_timer_bit_t) resolutionBits;
        ledc_timer.freq_hz = frequency;
        ledc_timer.speed_mode = mode();
        ledc_timer.timer_num = mTimerId;

        ledc_timer_config(&ledc_timer);
    }

    ledc_channel_t nextChannel()
    {
        auto ret = (ledc_channel_t) mCurrentChannel;
        ++mCurrentChannel;
        return ret;
    }

    ledc_timer_t id() const
    {
        return mTimerId;
    }

    uint32_t steps() const
    {
        return mSteps;
    }

    ledc_mode_t mode() const
    {
        return LEDC_HIGH_SPEED_MODE;
    }

    float frequency() const
    {
        return mFrequency;
    }
private:
    ledc_timer_t const mTimerId;
    int mCurrentChannel;
    uint32_t mSteps;
    float mFrequency;
};

struct PWM
{
    PWM(PWMTimer * timer, gpio_num_t pin) :
        mTimer(timer),
        mChannel(timer->nextChannel())
    {
        ledc_channel_config_t pwm_config;
        pwm_config.channel = mChannel;
        pwm_config.duty = 0;
        pwm_config.gpio_num = pin;
        pwm_config.speed_mode = timer->mode();
        pwm_config.hpoint = 0;
        pwm_config.timer_sel = timer->id();

        ledc_channel_config(&pwm_config);
    }

    void duty(float duty)
    {
        auto steps = calculateDuty(duty);
        ledc_set_duty(mTimer->mode(), mChannel, steps);
        ledc_update_duty(mTimer->mode(), mChannel);
    }

protected:
    uint32_t calculateDuty(float duty) const {
        return static_cast<uint32_t>(mTimer->steps() * duty);
    }

    PWMTimer * const mTimer;
    ledc_channel_t const mChannel;
};