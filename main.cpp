#include "WiFi.hpp"
#include "Flyer.hpp"
#include "esp_log.h"

static void flyerTask(void *)
{
    Flyer flyer;
    flyer.run();
}

extern "C" void app_main()
{
    initWiFi([]() {
        xTaskCreatePinnedToCore(flyerTask, "flyerTask", 1024 * 16, (void *) 0, 10, NULL, 1);
    });
}
