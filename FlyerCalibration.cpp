#include "Flyer.hpp"

#include <icarus/sensor/EllipsoidalCalibrator.hpp>
#include <icarus/sensor/VarianceEstimator.hpp>

#include <esp_log.h>

static constexpr auto TAG = "Flyer";

void Flyer::fullCalibration(Calibration & calibration)
{
    ESP_LOGI(TAG, "Calibrating magnetometer, shake it!");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    calibrateMagnetometer(calibration);

    ESP_LOGI(TAG, "Calibrating gyroscope, stop it!");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    calibrateGyroscope(calibration);
}

void Flyer::calibrateMagnetometer(Calibration & calibration)
{
    constexpr size_t sampleSize = 100;

    icarus::EllipsoidalCalibrator<float> magCal(sampleSize);
    icarus::EllipsoidalCalibrator<float> accCal(sampleSize);

    for (int i = 0; i < sampleSize; ++i) {
        magCal.addSample(mIMU.rawMagneticField());
        accCal.addSample(mIMU.rawAcceleration());
        yield();
    }

    ESP_LOGI(TAG, "Computing magneto calibration");

    calibration.waveshare.magnetometer = magCal.computeCalibration(1.0f);
    calibration.waveshare.accelerometer = accCal.computeCalibration(9.81f);

    Eigen::Matrix<float, 3, 3> axes;
    axes << 0, 1, 0,
            1, 0, 0,
            0, 0, -1;

    calibration.waveshare.magnetometer.transformAxes(axes);
}

void Flyer::calibrateGyroscope(Calibration & calibration)
{
    constexpr size_t sampleSize = 100;

    icarus::VarianceEstimator<float, 3> gyroVar(100), magVar(100), accVar(100);

    for (int i = 0; i < sampleSize; ++i) {
        gyroVar.addSample(mIMU.rawAngularVelocity());
        magVar.addSample(mIMU.magneticField());
        accVar.addSample(mIMU.acceleration());
        yield();
    }

    calibration.waveshare.gyroscope = icarus::OffsetCalibration<float, 3>(gyroVar.mean());
    calibration.gyroscopeVariance = gyroVar.variance();
    calibration.magnetometerVariance = magVar.variance();

    Eigen::Vector3f accOffset = accVar.mean();
    accOffset -= Eigen::Vector3f(0.0f, 0.0f, 9.81f);

    calibration.waveshare.accelerometer.addOffset(-accOffset);
    calibration.accelerometerVariance = accVar.variance();
}
