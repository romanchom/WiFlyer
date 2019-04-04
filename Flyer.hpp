#pragma once

#include "I2CBus.hpp"
#include "Waveshare10DOF.hpp"

struct Flyer {
    Flyer();
    void run();
private:
    void step();

    void initialize();
    void calibrate();
    void track();

    using Phase = void (Flyer::*)();
    Phase mPhase;
    TickType_t mNextStepTime;

    I2CBus mI2C;

    Waveshare10DOF mIMU;
};
