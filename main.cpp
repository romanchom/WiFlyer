#include "WiFi.hpp"
#include "I2CBus.hpp"
#include "EspWait.hpp"
#include "ProtoBoardConfig.hpp"

#include <icarus/MPU9255.hpp>
#include <icarus/BMP180.hpp>
#include <icarus/AK8963.hpp>
#include <icarus/I2CRegisterBank.hpp>
#include <icarus/EllipsoidalCalibrator.hpp>

#include "esp_log.h"

#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>

constexpr auto TAG = "main";

struct Telemetry {
    Eigen::Vector3f acceleration;
    Eigen::Vector3f angularVelocity;
    Eigen::Vector3f magneticField;
    float pressure;
    float temperature;
};


static void sensorTask(void *arg)
{
    auto i2c = I2CBus(gpioSda, gpioScl);

    auto mpu9255Device = icarus::I2CRegisterBank(&i2c, 104);
    auto mpu9255 = icarus::MPU9255(&mpu9255Device);

    auto ak8963Device = icarus::I2CRegisterBank(&i2c, 12);
    auto ak8963 = icarus::AK8963<EspWait, decltype(ak8963Device)>(&ak8963Device);

    auto bmp180Device = icarus::I2CRegisterBank(&i2c, 119);
    auto bmp180 = icarus::BMP180(&bmp180Device);

    while (true) {
        try {
            mpu9255.initialize();
            mpu9255.i2cBypass(true);
            ak8963.initialize();
            bmp180.initialize();
            bmp180.startTemperatureRead();
            vTaskDelay(10 / portTICK_PERIOD_MS);
            bmp180.readTemperature();
            ESP_LOGI(TAG, "%f C\n\r", bmp180.temperature());
            break;
        } catch (std::runtime_error const & error){
            ESP_LOGI(TAG, "Error: %s", error.what());
        }
    }

    auto nextLoop = xTaskGetTickCount();

    struct sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = 0xFFFFFFFF;
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(1234);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }
    connect(sock, (struct sockaddr *) &destAddr, sizeof(destAddr));
    ESP_LOGI(TAG, "Socket created");

    while (true) {
        vTaskDelayUntil(&nextLoop, 10 / portTICK_PERIOD_MS);

        try {
            mpu9255.read();
            ak8963.read();
            bmp180.readPressure();
            bmp180.startPressureRead();
        } catch (std::runtime_error const & error) {
            ESP_LOGI(TAG, "%s", error.what());
            continue;
        }

        Telemetry telemetry = {
            mpu9255.acceleration(),
            mpu9255.angularVelocity(),
            ak8963.magneticField(),
            bmp180.pressure(),
            bmp180.temperature(),
        };

        send(sock, &telemetry, sizeof(telemetry), 0);
    }
}

template<typename Sampler>
icarus::EllipsoidalCalibration calibrate(Sampler const & sampler)
{
    icarus::EllipsoidalCalibrator cal(100);

    ESP_LOGI(TAG, "Reading samples");
    for (int i = 0; i < 100; ++i) {
        cal.addSample(sampler());
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "Beginning calibration");
    return cal.computeCalibration(9.8f);
}

static void calibrationTestTask(void*)
{

    auto i2c = I2CBus(gpioSda, gpioScl);

    auto mpu9255Device = icarus::I2CRegisterBank(&i2c, 104);
    auto mpu9255 = icarus::MPU9255(&mpu9255Device);

    mpu9255.initialize();

    auto accelCal = calibrate([&](){
        mpu9255.read();
        return mpu9255.acceleration();
    });
    ESP_LOGI(TAG, "Calibrated");

    while (true) {
        mpu9255.read();
        auto raw = mpu9255.acceleration();
        auto adj = accelCal.adjust(raw);
        ESP_LOGI(TAG, "Raw: %f %f %f", raw[0], raw[1], raw[2]);
        ESP_LOGI(TAG, "Adj: %f %f %f", adj[0], adj[1], adj[2]);

        vTaskDelay(30 / portTICK_PERIOD_MS);
    }
}


extern "C" void app_main()
{
    xTaskCreate(calibrationTestTask, "cal", 1024 * 16, (void *) 0, 10, NULL);

    // initWiFi([]() {
    //     xTaskCreate(sensorTask, "sensorTask", 1024 * 4, (void *) 0, 10, NULL);
    // });
}
