#include "WiFi.hpp"
#include "I2CBus.hpp"
#include "EspWait.hpp"
#include "ProtoBoardConfig.hpp"

#include <icarus/MPU9255.hpp>
#include <icarus/BMP180.hpp>
#include <icarus/AK8963.hpp>
#include <icarus/I2CRegisterBank.hpp>

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

extern "C" void app_main()
{
    initWiFi([]() {
        xTaskCreate(sensorTask, "sensorTask", 1024 * 4, (void *) 0, 10, NULL);
    });
}
