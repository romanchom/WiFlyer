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
    mTelemetry.angularVelocity = mMPU9255.angularVelocity() + mGyroOffset;
    mTelemetry.magneticField = mMagnetometerCalibration.adjust(mAK8964.magneticField());
    mTelemetry.pressure = mBMP180.pressure();

    vTaskDelayUntil(&mNextStepTime, 10 / portTICK_PERIOD_MS);
}

void Flyer::initialize()
{
    ESP_LOGI(TAG, "Initializing sensors");

    try {
        ESP_LOGI(TAG, "Initializing MPU9255");
        mMPU9255.initialize();
        mMPU9255.i2cBypass(true);
        ESP_LOGI(TAG, "Initializing AK8964");
        mAK8964.initialize();
        ESP_LOGI(TAG, "Initializing MPUBMP180");
        mBMP180.initialize();
        mPhase = &Flyer::calibrate;
    } catch (std::runtime_error const & error) {
        ESP_LOGI(TAG, "Error: %s", error.what());
    }
}

void Flyer::calibrate()
{
    ESP_LOGI(TAG, "Starting calibration");

    mNextStepTime = xTaskGetTickCount();

    // icarus::EllipsoidalCalibrator magCal(100);

    mGyroOffset.setZero();
    mGyroVariance.setZero();

    for (int i = 0; i < 100; ++i) {
        step();

        auto val = mMPU9255.angularVelocity();
        auto oldMean = mGyroOffset;
        mGyroOffset += (val - mGyroOffset) / (i + 1);
        mGyroVariance += (val - oldMean).cwiseProduct(val - mGyroOffset);
        // magCal.addSample(mAK8964.magneticField());
    }

    mGyroOffset *= -1;
    mGyroVariance /= 100;

    ESP_LOGI(TAG, "Gyro offset: %f %f %f",  mGyroOffset.x(), mGyroOffset.y(), mGyroOffset.z());
    ESP_LOGI(TAG, "Gyro variance: %f %f %f",  mGyroVariance.x(), mGyroVariance.y(), mGyroVariance.z());

    // mMagnetometerCalibration = magCal.computeCalibration(1.0f);

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

    icarus::RigidBodyProcessModel<float> processModel;
    icarus::GyroscopeMeasurementModel<float> measurementModel;

    mNextStepTime = xTaskGetTickCount();
    using CLK = std::chrono::high_resolution_clock;


    icarus::UnscentedKalmanFilter<float, 7> kalman;
    auto & state = kalman.state<icarus::RigidBodyProcessModel<float>::State>();
    state.orientation.setIdentity();
    state.angularMomentum.setZero();

    icarus::GaussianDistribution<float, 3> measurement;
    measurement.mean = mTelemetry.angularVelocity;
    measurement.covariance.setZero();
    measurement.covariance.diagonal() << mGyroVariance;

    while (true) {
        step();

        auto begin = CLK::now();
        kalman.filter(processModel, measurementModel, measurement, 0.01f);
        state.orientation.normalize();
        auto end = CLK::now();
        auto micros = std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(end - begin);
        ESP_LOGI(TAG, "%d", micros.count());

        mTelemetry.orientation = state.orientation;
        mTelemetry.angularMomentum = state.angularMomentum;

        send(sock, &mTelemetry, sizeof(mTelemetry), 0);
    }

    close(sock);
}
