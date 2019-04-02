#pragma once

#include <icarus/bus/I2CRegisterBank.hpp>
#include <icarus/sensor/MPU9255.hpp>
#include <icarus/sensor/BMP180.hpp>
#include <icarus/sensor/AK8963.hpp>
#include <icarus/sensor/EllipsoidalCalibration.hpp>
#include <icarus/sensor/OffsetCalibration.hpp>

#include "EspWait.hpp"
#include "I2CBus.hpp"

struct Waveshare10DOF
{
    explicit Waveshare10DOF(I2CBus * bus);

    void initialize();
    void calibrate();
    void read();

    Eigen::Matrix<float, 3, 1> acceleration() const;
    Eigen::Matrix<float, 3, 1> angularVelocity() const;
    Eigen::Matrix<float, 3, 1> angularVelocityVariance() const;
    Eigen::Matrix<float, 3, 1> magneticField() const;
    Eigen::Matrix<float, 3, 1> magneticFieldVariance() const;
private:
    void fullCalibration();
    void calibrateMagnetometer();
    void calibrateGyroscope();

    using RegisterBank = icarus::I2CRegisterBank<I2CBus>;

    RegisterBank mMPU9255Registers;
    RegisterBank mAK8963Register;
    RegisterBank mBMP180Register;

    icarus::MPU9255<RegisterBank> mMPU9255;
    icarus::AK8963<EspWait, RegisterBank> mAK8964;
    icarus::BMP180<RegisterBank> mBMP180;

    struct Calibration {
        icarus::EllipsoidalCalibration<float> accelerometer;
        Eigen::Matrix<float, 3, 1> accelerometerVariance;

        icarus::OffsetCalibration<float, 3> gyroscope;
        Eigen::Matrix<float, 3, 1> gyroscopeVariance;

        icarus::EllipsoidalCalibration<float> magnetometer;
        Eigen::Matrix<float, 3, 1> magnetometerVariance;
    };

    Calibration mCalibration;
};
