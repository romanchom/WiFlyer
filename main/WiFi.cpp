#include "WiFi.hpp"

#include <string.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>

#define TAG "WiFi"

WiFi::WiFi()
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(&WiFi::eventHandler, this));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {};
    strncpy((char*) wifi_config.sta.ssid, CONFIG_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*) wifi_config.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
}

void WiFi::start()
{
    ESP_ERROR_CHECK(esp_wifi_start());
}

void WiFi::addListener(WiFiListener * listener)
{
    mListeners.push_back(listener);
}

int WiFi::eventHandler(void * context, system_event_t * event)
{
    return static_cast<WiFi *>(context)->handleEvent(event);
}

int WiFi::handleEvent(system_event_t * event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(TAG, "Connecting");
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "Disconnected");
        for (auto * listener : mListeners) {
            listener->onWiFiDisconnected();
        }
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        for (auto * listener : mListeners) {
            listener->onWiFiConnected();
        }
        break;
    }
    return ESP_OK;
}

