#include "Waveshare10DOF.hpp"

#include "EspWait.hpp"
#include "I2CBus.hpp"
#include "TaskLoop.hpp"

#include <icarus/sensor/MPU9255_impl.hpp>
#include <icarus/sensor/BMP180_impl.hpp>
#include <icarus/sensor/AK8963_impl.hpp>
#include <icarus/sensor/EllipsoidalCalibrator.hpp>
#include <icarus/sensor/VarianceEstimator.hpp>

#include <esp_log.h>
#include <nvs_flash.h>


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

void Waveshare10DOF::calibrate()
{
    nvs_handle handle;
    esp_err_t err;
    char const * const storageNamespace = "icarus";

    err = nvs_open(storageNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        throw std::runtime_error("Unable to access NVS");
    }

    size_t requiredSize = sizeof(mCalibration);
    err = nvs_get_blob(handle, "calibration", &mCalibration, &requiredSize);
    if (err != ESP_OK) {
        fullCalibration();

        ESP_LOGI(TAG, "Saving magneto calibration");

        err = nvs_set_blob(handle, "calibration", &mCalibration, requiredSize);
        err = nvs_commit(handle);
    }

    nvs_close(handle);
}

void Waveshare10DOF::fullCalibration()
{
    ESP_LOGI(TAG, "Calibrating magnetometer, shake it!");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    calibrateMagnetometer();

    ESP_LOGI(TAG, "Calibrating gyroscope, stop it!");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    calibrateGyroscope();
}

void Waveshare10DOF::calibrateMagnetometer()
{
    TaskLoop loop(10);
    constexpr size_t sampleSize = 100;

    icarus::EllipsoidalCalibrator<float> magCal(sampleSize);

    for (int i = 0; i < sampleSize; ++i) {
        read();
        magCal.addSample(mAK8964.magneticField());
        loop.wait();
    }

    ESP_LOGI(TAG, "Computing magneto calibration");

    mCalibration.magnetometer = magCal.computeCalibration(1.0f);
    Eigen::Matrix<float, 3, 3> axes;
    axes << 0, 1, 0,
            1, 0, 0,
            0, 0, -1;

    mCalibration.magnetometer.transformAxes(axes);
}

void Waveshare10DOF::calibrateGyroscope()
{
    TaskLoop loop(100);
    constexpr size_t sampleSize = 100;

    icarus::VarianceEstimator<float, 3> gyroVar(100), magVar(100);

    for (int i = 0; i < sampleSize; ++i) {
        read();
        gyroVar.addSample(mMPU9255.angularVelocity());
        magVar.addSample(magneticField());
        loop.wait();
    }

    mCalibration.gyroscope = icarus::OffsetCalibration<float, 3>(gyroVar.mean());
    mCalibration.gyroscopeVariance = gyroVar.variance();
    mCalibration.magnetometerVariance = magVar.variance();
}

void Waveshare10DOF::read()
{
    mMPU9255.read();
    mAK8964.read();
    mBMP180.read();
}

Eigen::Matrix<float, 3, 1> Waveshare10DOF::acceleration() const
{
    return mCalibration.accelerometer.adjust(mMPU9255.acceleration());
}

Eigen::Matrix<float, 3, 1> Waveshare10DOF::angularVelocity() const
{
    return mCalibration.gyroscope.adjust(mMPU9255.angularVelocity());
}

Eigen::Matrix<float, 3, 1> Waveshare10DOF::angularVelocityVariance() const
{
    return mCalibration.gyroscopeVariance;
}

Eigen::Matrix<float, 3, 1> Waveshare10DOF::magneticField() const
{
    return mCalibration.magnetometer.adjust(mAK8964.magneticField());
}

Eigen::Matrix<float, 3, 1> Waveshare10DOF::magneticFieldVariance() const
{
    return mCalibration.magnetometerVariance;
}

