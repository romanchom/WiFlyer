#pragma once

#include "I2CBus.hpp"

#include <icarus/I2CRegisterBank.hpp>
#include <icarus/MPU9255.hpp>
#include <icarus/BMP180.hpp>
#include <icarus/AK8963.hpp>
#include <icarus/EllipsoidalCalibration.hpp>

#include "EspWait.hpp"

#include "Telemetry.hpp"

struct Flyer {
    Flyer();
    void run();
private:
    void step();

    void initialize();
    void calibrate();
    void track();

    using RegisterBank = icarus::I2CRegisterBank<I2CBus>;

    using Phase = void (Flyer::*)();
    Phase mPhase;
    TickType_t mNextStepTime;

    I2CBus mI2C;

    RegisterBank mMPU9255Registers;
    RegisterBank mAK8963Register;
    RegisterBank mBMP180Register;

    icarus::MPU9255<RegisterBank> mMPU9255;
    icarus::AK8963<EspWait, RegisterBank> mAK8964;
    icarus::BMP180<RegisterBank> mBMP180;

    icarus::EllipsoidalCalibration mMagnetometerCalibration;

    Telemetry mTelemetry;
};
