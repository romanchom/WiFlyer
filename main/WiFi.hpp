#pragma once

#include <esp_event.h>

#include <vector>

struct WiFiListener
{
    virtual void onWiFiConnected() = 0;
    virtual void onWiFiDisconnected() = 0;
};

struct WiFi
{
    explicit WiFi();
    void start();
    void addListener(WiFiListener * listener);
private:
    static int eventHandler(void * context, system_event_t * event);
    int handleEvent(system_event_t * event);

    std::vector<WiFiListener *> mListeners;
};
