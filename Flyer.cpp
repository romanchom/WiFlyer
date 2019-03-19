#include "Flyer.hpp"

#include "ProtoBoardConfig.hpp"

#include <icarus/EllipsoidalCalibrator.hpp>

#include <icarus/UnscentedKalmanFilter.hpp>
#include <icarus/GyroscopeMeasurementModel.hpp>
#include <icarus/RigidBodyProcessModel.hpp>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>

#include <chrono>

constexpr auto TAG = "Flyer";

Flyer::Flyer() :
    mI2C(gpioSda, gpioScl),
    mMPU9255Registers(&mI2C, 104),
    mAK8963Register(&mI2C, 12),
    mBMP180Register(&mI2C, 119),
    mMPU9255(&mMPU9255Registers),
    mAK8964(&mAK8963Register),
    mBMP180(&mBMP180Register)
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
    mMPU9255.read();
    mAK8964.read();
    mBMP180.read();

    mTelemetry.acceleration = mMPU9255.acceleration();
    mTelemetry.angularVelocity = mMPU9255.angularVelocity();
    mTelemetry.magneticField = mMagnetometerCalibration.adjust(mAK8964.magneticField());
    mTelemetry.pressure = mBMP180.pressure();

    vTaskDelayUntil(&mNextStepTime, 10 / portTICK_PERIOD_MS);
}

void Flyer::initialize()
{
    ESP_LOGI(TAG, "Initializing calibration");

    try {
        mMPU9255.initialize();
        mMPU9255.i2cBypass(true);
        mAK8964.initialize();
        mBMP180.initialize();
        mPhase = &Flyer::track;
    } catch (std::runtime_error const & error) {
        ESP_LOGI(TAG, "Error: %s", error.what());
    }
}

void Flyer::calibrate()
{
    ESP_LOGI(TAG, "Starting calibration");

    mNextStepTime = xTaskGetTickCount();

    icarus::EllipsoidalCalibrator magCal(100);

    for (int i = 0; i < 100; ++i) {
        for (int j = 0; j < 10; ++j) {
            step();
        }

        magCal.addSample(mAK8964.magneticField());
    }

    ESP_LOGI(TAG, "Computing calibration");

    mMagnetometerCalibration = magCal.computeCalibration(1.0f);

    ESP_LOGI(TAG, "Calibration complete");

    mPhase = &Flyer::track;
}

void Flyer::track()
{
    sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = 0xFFFFFFFF;
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(1234);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }
    connect(sock, (sockaddr *) &destAddr, sizeof(destAddr));


    icarus::GaussianDistribution<float, 7> state;
    state.mean << 0, 0, 0, 0, 0, 0, 1;
    state.covariance.setIdentity();
    state.covariance *= 0.01f;

    icarus::RigidBodyProcessModel<float> processModel;
    icarus::GyroscopeMeasurementModel<float> measurementModel;

    using CLK = std::chrono::high_resolution_clock;

    while (true) {
        step();

        auto begin = CLK::now();

        icarus::GaussianDistribution<float, 3> measurement;
        measurement.mean = mTelemetry.angularVelocity;
        measurement.covariance = Eigen::Matrix<float, 3, 3>::Identity() * 0.00001f;

        state = UnscentedKalmanFilter(state, processModel, measurementModel, measurement);
        auto & s = reinterpret_cast<icarus::RigidBodyProcessModel<float>::State &>(state.mean);
        s.orientation.normalize();

        auto end = CLK::now();

        std::chrono::duration<float, std::milli> seconds = end - begin;

        mTelemetry.orientation = s.orientation;

        send(sock, &mTelemetry, sizeof(mTelemetry), 0);
    }

    close(sock);
}
