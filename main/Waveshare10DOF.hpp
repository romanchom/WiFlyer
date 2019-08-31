#pragma once

#include "EspWait.hpp"
#include "I2CBus.hpp"

#include <icarus/bus/I2CRegisterBank.hpp>
#include <icarus/sensor/MPU9255.hpp>
#include <icarus/sensor/BMP180.hpp>
#include <icarus/sensor/AK8963.hpp>
#include <icarus/sensor/EllipsoidalCalibration.hpp>
#include <icarus/sensor/OffsetCalibration.hpp>


struct Waveshare10DOF
{
    struct CalibrationData {
        // icarus::EllipsoidalCalibration<float> accelerometer;
        icarus::OffsetCalibration<float, 3> accelerometer;
        icarus::OffsetCalibration<float, 3> gyroscope;
        icarus::EllipsoidalCalibration<float> magnetometer;
    };

    explicit Waveshare10DOF(I2CBus * bus);

    void initialize();
    void read();
    void calibrationData(CalibrationData const & calibration);

    Eigen::Matrix<float, 3, 1> rawAcceleration() const;
    Eigen::Matrix<float, 3, 1> rawAngularVelocity() const;
    Eigen::Matrix<float, 3, 1> rawMagneticField() const;

    Eigen::Matrix<float, 3, 1> acceleration() const;
    Eigen::Matrix<float, 3, 1> angularVelocity() const;
    Eigen::Matrix<float, 3, 1> magneticField() const;
    float pressure() const;
    float temperature() const;

private:
    using RegisterBank = icarus::I2CRegisterBank<I2CBus>;

    RegisterBank mMPU9255Registers;
    RegisterBank mAK8963Register;
    RegisterBank mBMP180Register;

    icarus::MPU9255<RegisterBank> mMPU9255;
    icarus::AK8963<EspWait, RegisterBank> mAK8964;
    icarus::BMP180<RegisterBank> mBMP180;

    CalibrationData mCalibration;
};
