#include "Flyer.hpp"

#include <icarus/sensor/EllipsoidalCalibrator.hpp>
#include <icarus/sensor/VarianceEstimator.hpp>

#include <esp_log.h>
#include <nvs_flash.h>

#include <boost/range/irange.hpp>

using boost::irange;

static constexpr auto TAG = "Flyer";

void Flyer::calibrate()
{
    Calibration calibration;

    nvs_handle handle;
    esp_err_t err;

    err = nvs_open("icarus", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        // TODO fallback to full calibration if NVS is missing
        throw std::runtime_error("Unable to access NVS");
    }

    size_t requiredSize = sizeof(calibration);
    err = nvs_get_blob(handle, "calibration", &calibration, &requiredSize);
    if (err != ESP_OK) {
        fullCalibration(calibration);

        ESP_LOGI(TAG, "Saving magneto calibration");

        requiredSize = sizeof(calibration);
        err = nvs_set_blob(handle, "calibration", &calibration, requiredSize);
        err = nvs_commit(handle);
    }

    nvs_close(handle);

    mMeasurement.covariance.setZero();
    mMeasurement.covariance.diagonal() <<
        calibration.gyroscopeVariance * 10,
        calibration.magnetometerVariance * 10,
        calibration.accelerometerVariance * 10,
        25;

    mIMU.calibrationData(calibration.waveshare);
}

void Flyer::fullCalibration(Calibration & calibration)
{
    ESP_LOGI(TAG, "Calibrating magnetometer, shake it!");
    for (auto i : irange(100)) {
        yield();
    }

    calibrateMagnetometer(calibration);

    ESP_LOGI(TAG, "Calibrating gyroscope, stop it!");
    for (auto i : irange(500)) {
        yield();
    }

    calibrateGyroscope(calibration);
}

void Flyer::calibrateMagnetometer(Calibration & calibration)
{
    constexpr size_t sampleSize = 100;

    icarus::EllipsoidalCalibrator<float> magCal(sampleSize);
    // icarus::EllipsoidalCalibrator<float> accCal(sampleSize);

    for (auto i : irange(sampleSize)) {
        for (auto j : irange(10)) {
            yield();
        }
        magCal.addSample(mIMU.rawMagneticField());
        // accCal.addSample(mIMU.rawAcceleration());
    }

    ESP_LOGI(TAG, "Computing magneto calibration");

    calibration.waveshare.magnetometer = magCal.computeCalibration(1.0f);
    // calibration.waveshare.accelerometer = accCal.computeCalibration(9.81f);

    Eigen::Matrix<float, 3, 3> axes;
    axes << 0, 1, 0,
            1, 0, 0,
            0, 0, -1;

    calibration.waveshare.magnetometer.transformAxes(axes);
}

void Flyer::calibrateGyroscope(Calibration & calibration)
{
    constexpr size_t sampleSize = 100;

    icarus::VarianceEstimator<float, 3> gyroVar(sampleSize), magVar(sampleSize), accVar(sampleSize);

    for (auto i : irange(sampleSize)) {
        yield();
        gyroVar.addSample(mIMU.rawAngularVelocity());
        magVar.addSample(calibration.waveshare.magnetometer.adjust(mIMU.rawMagneticField()));
        accVar.addSample(calibration.waveshare.accelerometer.adjust(mIMU.rawAcceleration()));
    }

    calibration.waveshare.gyroscope = icarus::OffsetCalibration<float, 3>(gyroVar.mean());
    calibration.waveshare.accelerometer = icarus::OffsetCalibration<float, 3>(accVar.mean() - Eigen::Vector3f(0.0f, 0.0f, 9.81f));

    calibration.gyroscopeVariance = gyroVar.variance();
    calibration.magnetometerVariance = magVar.variance();

    // Eigen::Vector3f accOffset = accVar.mean();
    // accOffset -= Eigen::Vector3f(0.0f, 0.0f, 9.81f);

    // calibration.waveshare.accelerometer.addOffset(-accOffset);
    calibration.accelerometerVariance = accVar.variance();
}
