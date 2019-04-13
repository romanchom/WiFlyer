#pragma once

#include <algorithm>

template<typename T>
struct PID
{
    explicit PID(T maxError, T kP, T kI = 0, T kD = 0) :
        mMaximumError(maxError),
        mKP(kP),
        mKI(kI),
        mKD(kD),
        mAccumulatedError(0),
        mPreviousError(0)
    {}

    T control(T error, T timeStep)
    {
        T signal = 0;
        signal += mKP * error;
        signal += mKI * mAccumulatedError;
        signal += mKD * (mPreviousError - error) / timeStep;
        mPreviousError = error;
        mAccumulatedError = std::clamp(mAccumulatedError + error * timeStep, -mMaximumError, mMaximumError);
        return signal;
    }

private:
    T mMaximumError;
    T mKP;
    T mKI;
    T mKD;
    T mAccumulatedError;
    T mPreviousError;
};
