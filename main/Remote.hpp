#pragma once

#include "WiFi.hpp"

#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>


struct Remote : WiFiListener
{
    explicit Remote();
    void write(std::byte const * data, size_t size);
    size_t read(std::byte * data, size_t size);

    void onWiFiConnected() override;
    void onWiFiDisconnected() override;
private:
    int mDescriptor;
};
