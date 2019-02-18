#include "WiFi.hpp"
#include "I2CBus.hpp"
#include "EspWait.hpp"
#include "ProtoBoardConfig.hpp"

#include <icarus/MPU9255.hpp>
#include <icarus/BMP180.hpp>
#include <icarus/AK8963.hpp>
#include <icarus/I2CRegisterBank.hpp>

#include "esp_log.h"


constexpr auto TAG = "main";

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
            vTaskDelay(1);
            bmp180.readTemperature();
            ESP_LOGI(TAG, "%f C\n\r", bmp180.temperature());
            break;
        } catch (std::runtime_error const & error){
            ESP_LOGI(TAG, "Error: %s", error.what());
        }
    }

    auto nextLoop = xTaskGetTickCount();

    while (true) {
        try {
            mpu9255.read();
            ak8963.read();
            bmp180.readPressure();
            bmp180.startPressureRead();
            // ESP_LOGI(TAG, "%f Pa", bmp180.pressure());

        } catch (std::runtime_error const & error) {
            ESP_LOGI(TAG, "%s", error.what());
        }

        vTaskDelayUntil(&nextLoop, 10 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main()
{
    wifi_init_sta();

    xTaskCreate(sensorTask, "sensorTask", 1024 * 4, (void *) 0, 10, NULL);
}
