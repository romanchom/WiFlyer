#pragma once

#include "I2CBus.hpp"
#include "Waveshare10DOF.hpp"
#include "Telemetry.hpp"

#include <icarus/sensorFusion/UnscentedKalmanFilter.hpp>
#include <icarus/sensorFusion/FlightModel.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct Flyer {
    Flyer();
    void run();
private:
    struct Calibration {
        Waveshare10DOF::CalibrationData waveshare;
        Eigen::Matrix<float, 3, 1> accelerometerVariance;
        Eigen::Matrix<float, 3, 1> gyroscopeVariance;
        Eigen::Matrix<float, 3, 1> magnetometerVariance;
    };

    static void i2cTask(void * context);
    static void stabilizationTask(void * context);

    void communicateI2C();

    void stabilize();
    void readOutSensors();
    void estimateState();

    void yield();

    void control();
    void zeroOutState();
    void calibrate();
    void fullCalibration(Calibration & calibration);
    void calibrateMagnetometer(Calibration & calibration);
    void calibrateGyroscope(Calibration & calibration);

    void fly();

    TaskHandle_t mControllerTask;
    TaskHandle_t mStabilizationTask;
    TaskHandle_t mI2CTask;

    TickType_t mNextStepTime;

    I2CBus mI2C;

    Waveshare10DOF mIMU;


    using FlightModel = icarus::FlightModel<float>;

    FlightModel::ProcessModel mProcessModel;
    FlightModel::MeasurementModel mMeasurementModel;

    icarus::UnscentedKalmanFilter<float, 16> mFilter;
    icarus::GaussianDistribution<float, 7> mMeasurement;

    Telemetry mTelemetry;
};
