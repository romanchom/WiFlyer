#include "Flyer.hpp"
#include "esp_log.h"

#include <nvs_flash.h>

static void flyerTask(void *)
{
    Flyer flyer;
    flyer.run();
}

extern "C" void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    xTaskCreatePinnedToCore(flyerTask, "flyerTask", 1024 * 16, (void *) 0, 10, NULL, 1);
}
