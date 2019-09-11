#include "Flyer.hpp"

#include "ProtoBoardConfig.hpp"
#include "TaskNotification.hpp"
#include "Messages.hpp"

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
    mIMU(&mI2C),
    mMotorsEnabled(false),
    mAxialError(Eigen::Vector3f::Zero()),
    mMotorTimer(0, 200, 12),
    mMotors{
        {&mMotorTimer, gpioEscStarboardBow},
        {&mMotorTimer, gpioEscPortBow},
        {&mMotorTimer, gpioEscPortQuarter},
        {&mMotorTimer, gpioEscStarboardQuarter},
    },
    mSonar(32, 39),
    mRollPID(0.1f, 0.0f),
    mPitchPID(0.1, 0.0f),
    mYawPID(0.05f, 0.0f),
    mThrottle(0.0f)
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
    mMotorsEnabled = false;
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
        mSonar.startMeasurement();

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
        xTaskNotifyGive(mControllerTask);

        // wait for controller
        notification.waitForEvent(taskEvent::controller);

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
    std::array<float, 4> thrusts;

    float xThrust = mRollPID.control(mAxialError[1], periodSeconds);
    float yThrust = mPitchPID.control(-mAxialError[0], periodSeconds);

    thrusts[0] = -xThrust - yThrust;
    thrusts[1] = xThrust - yThrust;
    thrusts[2] = xThrust + yThrust;
    thrusts[3] = -xThrust + yThrust;

    for (auto i : irange(4)) {
        thrusts[i] += mThrottle;
    }

    if (mMotorsEnabled) {
        for (auto i : irange(4)) {
            mMotors[i].thrust(std::clamp(thrusts[i], 0.08f, 0.4f));
        }
    } else {
        for (auto i : irange(4)) {
            mMotors[i].thrust(0.0f);
        }
    }

    mTelemetry.motors = thrusts;
}

void Flyer::yield()
{
    xTaskNotify(mStabilizationTask, taskEvent::controller, eSetBits);
    ulTaskNotifyTake(true, portMAX_DELAY);
}

void Flyer::control()
{
    ulTaskNotifyTake(true, portMAX_DELAY);
    for (auto i : irange(20)) {
        // wait untill sensors stabilize
        yield();
    }
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

void Flyer::takeOff()
{
    mMotorsEnabled = true;
    mRollPID.reset();
    mPitchPID.reset();

    mFilter.reset();
    auto state = FlightModel::State(mFilter.stateVector());
    state.reset();
}

void Flyer::fly()
{
    for (;;) {
        std::byte buffer[64];
        auto bytesRead = mRemote.read(buffer, sizeof(buffer));
        if (bytesRead > 0) {
            switch (int(buffer[0])) {
            case PIDMessage::id:
                {
                    auto & m = *reinterpret_cast<PIDMessage *>(buffer + 1);
                    mRollPID.set(m.rollP, m.rollI, m.rollD);
                    mPitchPID.set(m.pitchP, m.pitchI, m.pitchD);
                }
                break;
            case SteeringMessage::id:
                {
                    auto & m = *reinterpret_cast<SteeringMessage *>(buffer + 1);
                    if (!mMotorsEnabled && (m.enabled != 0)) {
                        takeOff();
                    }
                    mMotorsEnabled = (m.enabled != 0);
                    mThrottle = m.throttle;
                }
                break;
            }
        }

        auto state = FlightModel::State(mFilter.stateVector());

        Eigen::Quaternion<float> targetOrientation;
        targetOrientation.setIdentity();

        Eigen::AngleAxis<float> deltaOrientation(targetOrientation * state.orientation().conjugate());

        mAxialError = deltaOrientation.axis() * deltaOrientation.angle();

        yield();
    }
}
