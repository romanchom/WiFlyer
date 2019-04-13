#pragma once

#include "PWM.hpp"

struct MotorPWM : PWM
{
    MotorPWM(PWMTimer * timer, gpio_num_t pin, float durationAtZeroPower = 0.001f, float durationAtFullPower = 0.002f) :
        PWM(timer, pin),
        mDurationAtZeroPower(durationAtZeroPower),
        mDurationAtFullPower(durationAtFullPower)
    {
        thrust(0);
    }

    void thrust(float fraction)
    {
        float duration = mDurationAtZeroPower * (1 - fraction) + mDurationAtFullPower * fraction;
        duty(duration * mTimer->frequency());
    }

protected:
    float mDurationAtZeroPower;
    float mDurationAtFullPower;
};
