#include "MotorPWM.hpp"

#include "Remote.hpp"
#include "WiFi.hpp"

#include <icarus/sensorFusion/UnscentedKalmanFilter.hpp>
#include <icarus/sensorFusion/FlightModel.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


struct QuadcopterPlatform
{
    explicit QuadcopterPlatform();
    void run();

private:
    static void i2cTask(void * context);
    void communicateI2C();

    void onWiFiConnected() override;
    void onWiFiDisconnected() override;

    TaskHandle_t mI2CTask;

    I2CBus mI2C;

    Waveshare10DOF mIMU;

    PWMTimer mMotorTimer;
    MotorPWM mMotors[4];
};