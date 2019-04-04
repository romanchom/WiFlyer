#include "Flyer.hpp"

#include "ProtoBoardConfig.hpp"
#include "Telemetry.hpp"

#include <icarus/sensorFusion/UnscentedKalmanFilter.hpp>
#include <icarus/sensorFusion/FlightModel.hpp>

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

    mNextStepTime = xTaskGetTickCount();
    for (int i = 0; i < 25; ++i) {
        step();
    }

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

    using FlightModel = icarus::FlightModel<float>;

    FlightModel::ProcessModel processModel;
    FlightModel::MeasurementModel measurementModel;

    Eigen::Vector3f referenceMagneticField;
    referenceMagneticField.setZero();
    float referencePressure = 0;

    constexpr int sampleSize = 10;
    for (int i = 0; i < sampleSize; ++i) {
        step();
        referenceMagneticField += mIMU.magneticField();
        referencePressure += mIMU.pressure();
    }

    referenceMagneticField /= sampleSize;
    referencePressure /= sampleSize;

    measurementModel.referenceMagneticField(referenceMagneticField);
    measurementModel.referencePressure(referencePressure);

    mNextStepTime = xTaskGetTickCount();
    using CLK = std::chrono::high_resolution_clock;

    icarus::UnscentedKalmanFilter<float, 16> kalman;
    auto & state = kalman.state<FlightModel::State>();
    state.orientation.setIdentity();
    state.angularMomentum.setZero();
    state.position.setZero();
    state.velocity.setZero();
    state.acceleration.setZero();

    icarus::GaussianDistribution<float, 10> measurement;
    measurement.covariance.setZero();
    measurement.covariance.diagonal() <<
        mIMU.angularVelocityVariance(),
        mIMU.magneticFieldVariance() * 100,
        mIMU.accelerationVariance() * 100,
        1600;

    // std::cout << "Measurement covariance " << measurement.covariance.diagonal().transpose() << std::endl;

    // int i = 0;
    while (true) {
        step();

        auto begin = CLK::now();

        measurement.mean <<
            mIMU.angularVelocity(),
            mIMU.magneticField(),
            mIMU.acceleration(),
            mIMU.pressure();

        kalman.filter(processModel, measurementModel, measurement, 0.01f);
        state.orientation.normalize();

        auto end = CLK::now();
        auto micros = std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(end - begin);

        // if (i == 0) {
        //     ESP_LOGI(TAG, "T = %d", micros.count());
        //     i = 100;
        // }
        // --i;

        Telemetry t;
        t.orientation = state.orientation;
        t.angularMomentum = state.angularMomentum;
        t.position = state.position;
        t.acceleration = state.acceleration;
        t.angularVelocity = mIMU.angularVelocity();
        t.magneticField = mIMU.magneticField();
        t.pressure = mIMU.pressure();

        send(sock, &t, sizeof(t), 0);
    }

    close(sock);
}
