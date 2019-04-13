#include "Flyer.hpp"

#include "ProtoBoardConfig.hpp"

#include <esp_log.h>
#include <nvs_flash.h>

#include <iostream>
#include <chrono>

enum taskEvent : uint32_t {
    i2c = 1 << 0,
    kalman = 1 << 1,
    pid = 1 << 2,
    controller = 1 << 3,
};

static constexpr auto TAG = "Flyer";

Flyer::Flyer() :
    mI2C(gpioSda, gpioScl),
    mIMU(&mI2C)
{
    mMeasurement.covariance.setIdentity();

    xTaskCreatePinnedToCore(&Flyer::i2cTask, "i2c", 256, this, configMAX_PRIORITIES - 1, &mI2CTask, 1);
    xTaskCreatePinnedToCore(&Flyer::stabilizationTask, "i2c", 1024 * 4, this, 0, &mStabilizationTask, 0);
}

void Flyer::run()
{
    mControllerTask = xTaskGetCurrentTaskHandle();
    mNextStepTime = xTaskGetTickCount();

    control();
}

void Flyer::i2cTask(void * context)
{
    try {
        static_cast<Flyer *>(context)->communicateI2C();
    } catch (std::runtime_error const & error) {
        ESP_LOGE(TAG, "%s", error.what());
    }
    vTaskDelete(0);
}

void Flyer::stabilizationTask(void * context)
{
    try {
        static_cast<Flyer *>(context)->stabilize();
    } catch (std::runtime_error const & error) {
        ESP_LOGE(TAG, "%s", error.what());
    }
    vTaskDelete(0);
}

void Flyer::communicateI2C()
{
    mIMU.initialize();
    for (;;) {
        // i2c stuff
        mIMU.read();

        // wait for notification from stabilization task
        ulTaskNotifyTake(true, portMAX_DELAY);
    }
}


void Flyer::stabilize()
{
    uint32_t event;

    for (;;) {
        // pid and motor stuff
        // controlMotors();

        // wait for i2c
        while (event & taskEvent::i2c) {
            auto ret = xTaskNotifyWait(0, taskEvent::i2c, &event, pdMS_TO_TICKS(100));
            if (ret != pdPASS) {
                throw std::runtime_error("I2C not responding.");
            }
        }

        readOutSensors();
        estimateState();

        // notify controller
        xTaskNotifyGive(mControllerTask);

        // wait for controller
        while (event & taskEvent::controller) {
            auto ret = xTaskNotifyWait(0, taskEvent::controller, &event, pdMS_TO_TICKS(100));
            if (ret != pdPASS) {
                throw std::runtime_error("Controller not responding.");
            }
        }
    }
}

void Flyer::readOutSensors() {
    // get latest sensor values
    mMeasurement.mean <<
        mIMU.angularVelocity(),
        mIMU.magneticField(),
        // mIMU.acceleration(),
        mIMU.pressure();

    mTelemetry.angularVelocity = mIMU.angularVelocity();
    mTelemetry.magneticField = mIMU.magneticField();
    mTelemetry.pressure = mIMU.pressure();

    // start i2c task in the background
    xTaskNotifyGive(mI2CTask);
}

void Flyer::estimateState()
{
    mFilter.filter(mProcessModel, mMeasurementModel, mMeasurement, 0.01f);
    auto & state = mFilter.state<FlightModel::State>();
    state.orientation.normalize();

    mTelemetry.orientation = state.orientation;
    mTelemetry.angularMomentum = state.angularMomentum;
    mTelemetry.position = state.position;
    mTelemetry.acceleration = state.acceleration;
}

// void Flyer::controlMotors()
// {
//     // do stuff

//     // mTelemetry.motors = ???
// }

void Flyer::yield()
{
    vTaskDelayUntil(&mNextStepTime, pdMS_TO_TICKS(10));
    uint32_t event;
    auto ret = xTaskNotifyWait(0, taskEvent::kalman, &event, pdMS_TO_TICKS(100));
    if (ret == pdFALSE) {
        throw std::runtime_error("Stabilizer task not responding.");
    }
}

void Flyer::control()
{
    calibrate();
    zeroOutState();
    fly();
}

void Flyer::zeroOutState()
{
    Eigen::Vector3f referenceMagneticField;
    referenceMagneticField.setZero();
    float referencePressure = 0;

    constexpr int sampleSize = 10;
    for (int i = 0; i < sampleSize; ++i) {
        referenceMagneticField += mIMU.magneticField();
        referencePressure += mIMU.pressure();
        yield();
    }

    referenceMagneticField /= sampleSize;
    referencePressure /= sampleSize;

    mMeasurementModel.referenceMagneticField(referenceMagneticField);
    mMeasurementModel.referencePressure(referencePressure);

    auto & state = mFilter.state<FlightModel::State>();
    state.orientation.setIdentity();
    state.angularMomentum.setZero();
    state.position.setZero();
    state.velocity.setZero();
    state.acceleration.setZero();
}

void Flyer::calibrate()
{
    Calibration calibration;

    nvs_handle handle;
    esp_err_t err;
    char const * const storageNamespace = "icarus";

    err = nvs_open(storageNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        // TODO fallback to full calibration if NVS is missing
        throw std::runtime_error("Unable to access NVS");
    }

    size_t requiredSize = sizeof(calibration);
    err = nvs_get_blob(handle, "calibration", &calibration, &requiredSize);
    if (err != ESP_OK) {
        fullCalibration(calibration);

        ESP_LOGI(TAG, "Saving magneto calibration");

        err = nvs_set_blob(handle, "calibration", &calibration, requiredSize);
        err = nvs_commit(handle);
    }

    nvs_close(handle);

    mMeasurement.covariance.diagonal() <<
        calibration.accelerometerVariance,
        calibration.gyroscopeVariance * 100,
        calibration.magnetometerVariance * 100,
        1600;
}

void Flyer::fly()
{
    for (;;) {
        // do something with current state estimate and come up with motor values
        yield();
    }
}
