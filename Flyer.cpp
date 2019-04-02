#include "Flyer.hpp"

#include "ProtoBoardConfig.hpp"

#include <icarus/sensorFusion/UnscentedKalmanFilter.hpp>
#include <icarus/sensorFusion/GMMeasurementModel.hpp>
#include <icarus/sensorFusion/RigidBodyProcessModel.hpp>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>

#include <iostream>

#include <chrono>

constexpr auto TAG = "Flyer";

Flyer::Flyer() :
    mI2C(gpioSda, gpioScl),
    mIMU(&mI2C)
{}

void Flyer::run()
{
    mPhase = &Flyer::initialize;
    while (mPhase != nullptr) {
        (this->*mPhase)();
    }
}

void Flyer::step()
{
    mIMU.read();

    mTelemetry.acceleration = mIMU.acceleration();
    mTelemetry.angularVelocity = mIMU.angularVelocity();
    mTelemetry.magneticField = mIMU.magneticField();

    vTaskDelayUntil(&mNextStepTime, 10 / portTICK_PERIOD_MS);
}

void Flyer::initialize()
{
    ESP_LOGI(TAG, "Initializing sensors");

    try {
        mIMU.initialize();
        mPhase = &Flyer::calibrate;
    } catch (std::runtime_error const & error) {
        ESP_LOGI(TAG, "Error: %s", error.what());
    }
}

void Flyer::calibrate()
{
    ESP_LOGI(TAG, "Starting calibration");

    mIMU.calibrate();

    step();

    mPhase = &Flyer::track;
}

void Flyer::track()
{
    ESP_LOGI(TAG, "Tracking");

    sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = 0xFFFFFFFF;
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(12345);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }
    connect(sock, (sockaddr *) &destAddr, sizeof(destAddr));

    icarus::RigidBodyProcessModel<float> processModel;
    icarus::GMMeasurementModel<float> measurementModel;
    measurementModel.referenceMagneticField(mTelemetry.magneticField);

    mNextStepTime = xTaskGetTickCount();
    using CLK = std::chrono::high_resolution_clock;

    icarus::UnscentedKalmanFilter<float, 7> kalman;
    auto & state = kalman.state<icarus::RigidBodyProcessModel<float>::State>();
    state.orientation.setIdentity();
    state.angularMomentum.setZero();

    icarus::GaussianDistribution<float, 6> measurement;
    measurement.covariance.setZero();
    measurement.covariance.diagonal() << mIMU.angularVelocityVariance(), 0.01f, 0.01f, 0.01f;

    while (true) {
        step();

        measurement.mean << mTelemetry.angularVelocity, mTelemetry.magneticField;
        auto begin = CLK::now();
        kalman.filter(processModel, measurementModel, measurement, 0.01f);
        state.orientation.normalize();
        auto end = CLK::now();
        auto micros = std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(end - begin);
        // ESP_LOGI(TAG, "%d", micros.count());

        mTelemetry.orientation = state.orientation;
        mTelemetry.angularMomentum = state.angularMomentum;

        send(sock, &mTelemetry, sizeof(mTelemetry), 0);
    }

    close(sock);
}
