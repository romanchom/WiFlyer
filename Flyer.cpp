#include "Flyer.hpp"

#include "ProtoBoardConfig.hpp"
#include "TaskNotification.hpp"

#include <esp_log.h>
#include <nvs_flash.h>

#include <boost/range/irange.hpp>

#include <chrono>

using boost::irange;

namespace {
    enum taskEvent : uint32_t {
        i2c = 1 << 0,
        controller = 1 << 1,
    };

    constexpr auto TAG = "Flyer";
    constexpr int periodMilliseconds = 10;
    constexpr float periodSeconds = periodMilliseconds / 1000.0f;
}

Flyer::Flyer() :
    mI2C(gpioSda, gpioScl),
    mIMU(&mI2C)
{
    mMeasurement.covariance.setIdentity();

    mWiFi.addListener(this);
    mWiFi.addListener(&mRemote);
    mWiFi.start();
}



void Flyer::run()
{
    ESP_LOGI(TAG, "Running flyer main task.");

    mControllerTask = xTaskGetCurrentTaskHandle();

    ESP_LOGI(TAG, "Creating main task.");
    xTaskCreatePinnedToCore(&Flyer::i2cTask, "i2c", 1024 * 3, this, configMAX_PRIORITIES - 1, &mI2CTask, 1);

    ESP_LOGI(TAG, "Creating stabilization task.");
    xTaskCreatePinnedToCore(&Flyer::stabilizationTask, "stabilization", 1024 * 16, this, 5, &mStabilizationTask, 0);

    ESP_LOGI(TAG, "Entering control task.");
    control();
}

void Flyer::i2cTask(void * context)
{
    static_cast<Flyer *>(context)->communicateI2C();
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

void Flyer::onWiFiConnected()
{

}

void Flyer::onWiFiDisconnected()
{

}

void Flyer::communicateI2C()
try {
    mIMU.initialize();
    
    xTaskNotify(mStabilizationTask, taskEvent::i2c, eSetBits);
    // wait for notification from stabilization task
    ulTaskNotifyTake(true, portMAX_DELAY);

    for (;;) {
        // i2c stuff
        mIMU.read();

        xTaskNotify(mStabilizationTask, taskEvent::i2c, eSetBits);
        // wait for notification from stabilization task
        ulTaskNotifyTake(true, portMAX_DELAY);
    }
} catch (std::runtime_error const & error) {
    ESP_LOGE(TAG, "%s", error.what());
}

void Flyer::stabilize()
{
    ESP_LOGI(TAG, "S Enter.");
    TaskNotification notification;

    TickType_t nextStepTime = xTaskGetTickCount();
    for (;;) {
        controlMotors();

        // wait for i2c
        notification.waitForEvent(taskEvent::i2c);

        readOutSensors();
        estimateState();

        // notify controller
        // std::cout << "S->C" << std::endl;
        xTaskNotifyGive(mControllerTask);

        // wait for controller
        notification.waitForEvent(taskEvent::controller);
        // std::cout << "C->S" << std::endl;

        mRemote.write(reinterpret_cast<std::byte const *>(&mTelemetry), sizeof(mTelemetry));
        vTaskDelayUntil(&nextStepTime, pdMS_TO_TICKS(periodMilliseconds));
    }
}

void Flyer::readOutSensors() {
    // get latest sensor values
    auto measurement = FlightModel::Measurement(mMeasurement.mean);
    measurement.angularVelocity() = mIMU.angularVelocity();
    measurement.magneticField() = mIMU.magneticField();
    measurement.acceleration() = mIMU.acceleration();
    measurement.pressure() = mIMU.pressure();

    mTelemetry.acceleration = mIMU.acceleration();
    mTelemetry.angularVelocity = mIMU.angularVelocity();
    mTelemetry.magneticField = mIMU.magneticField();
    mTelemetry.pressure = mIMU.pressure();

    // start i2c task in the background
    xTaskNotifyGive(mI2CTask);
}

void Flyer::estimateState()
{
    mFilter.filter(mProcessModel, mMeasurementModel, mMeasurement, periodSeconds);
    auto state = FlightModel::State(mFilter.stateVector());
    state.orientation().normalize();

    mTelemetry.orientation = state.orientation();
    mTelemetry.angularMomentum = state.angularMomentum();
    mTelemetry.position = state.position();
    mTelemetry.velocity = state.velocity();
    mTelemetry.worldAcceleration = state.acceleration();
}

void Flyer::controlMotors()
{


    // mTelemetry.motors = ???
}

void Flyer::yield()
{
    xTaskNotify(mStabilizationTask, taskEvent::controller, eSetBits);
    ulTaskNotifyTake(true, portMAX_DELAY);
}

void Flyer::control()
{
    ulTaskNotifyTake(true, portMAX_DELAY);
    // for (auto i : irange(10)) {
        // wait untill sensors stabilize
        yield();
    // }
    ESP_LOGI(TAG, "Calibrating.");
    calibrate();
    ESP_LOGI(TAG, "Zeroing state.");
    zeroOutState();
    ESP_LOGI(TAG, "Flying.");
    fly();
}

void Flyer::zeroOutState()
{
    Eigen::Vector3f referenceMagneticField;
    referenceMagneticField.setZero();
    float referencePressure = 0;

    constexpr int sampleSize = 10;
    for (auto i : irange(sampleSize)) {
        yield();
        referenceMagneticField += mIMU.magneticField();
        referencePressure += mIMU.pressure();
    }

    referenceMagneticField /= sampleSize;
    referencePressure /= sampleSize;

    std::cout << "Ref mag: " << referenceMagneticField.transpose() << std::endl;
    std::cout << "Ref press: " << referencePressure << std::endl;

    mMeasurementModel.referenceMagneticField(referenceMagneticField);
    mMeasurementModel.referencePressure(referencePressure);

    mFilter.reset();
    auto state = FlightModel::State(mFilter.stateVector());
    state.reset();
}

void Flyer::fly()
{
    for (;;) {
        // do something with current state estimate and come up with motor values
        yield();
    }
}
