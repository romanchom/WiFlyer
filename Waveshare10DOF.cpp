#include "Waveshare10DOF.hpp"

#include <icarus/sensor/MPU9255_impl.hpp>
#include <icarus/sensor/BMP180_impl.hpp>
#include <icarus/sensor/AK8963_impl.hpp>

#include <esp_log.h>

constexpr auto TAG = "Waveshare";

Waveshare10DOF::Waveshare10DOF(I2CBus * bus) :
    mMPU9255Registers(bus, 104),
    mAK8963Register(bus, 12),
    mBMP180Register(bus, 119),
    mMPU9255(&mMPU9255Registers),
    mAK8964(&mAK8963Register),
    mBMP180(&mBMP180Register)
{}

void Waveshare10DOF::initialize()
{
    ESP_LOGI(TAG, "Initializing MPU9255");
    mMPU9255.initialize();
    mMPU9255.i2cBypass(true);
    ESP_LOGI(TAG, "Initializing AK8964");
    mAK8964.initialize();
    ESP_LOGI(TAG, "Initializing BMP180");
    mBMP180.initialize();
}

void Waveshare10DOF::read()
{
    mMPU9255.read();
    mAK8964.read();
    mBMP180.read();
}

void Waveshare10DOF::calibrationData(CalibrationData const & calibration)
{
    mCalibration = calibration;
}

Eigen::Matrix<float, 3, 1> Waveshare10DOF::rawAcceleration() const
{
    return mMPU9255.acceleration();
}

Eigen::Matrix<float, 3, 1> Waveshare10DOF::rawAngularVelocity() const
{
    return mMPU9255.angularVelocity();
}

Eigen::Matrix<float, 3, 1> Waveshare10DOF::rawMagneticField() const
{
    return mAK8964.magneticField();
}

Eigen::Matrix<float, 3, 1> Waveshare10DOF::acceleration() const
{
    return mCalibration.accelerometer.adjust(mMPU9255.acceleration());
}

Eigen::Matrix<float, 3, 1> Waveshare10DOF::angularVelocity() const
{
    return mCalibration.gyroscope.adjust(mMPU9255.angularVelocity());
}

Eigen::Matrix<float, 3, 1> Waveshare10DOF::magneticField() const
{
    return mCalibration.magnetometer.adjust(mAK8964.magneticField());
}

float Waveshare10DOF::pressure() const
{
    return mBMP180.pressure();
}

float Waveshare10DOF::temperature() const
{
    return mBMP180.temperature();
}
