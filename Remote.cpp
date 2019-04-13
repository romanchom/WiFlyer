#include "Remote.hpp"

#include "WiFi.hpp"

#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>

#include <esp_log.h>

char const * TAG = "Remote";

Remote::Remote() :
    mDescriptor(-1)
{}

void Remote::write(std::byte const * data, size_t size)
{
    if (mDescriptor != -1) {
        send(mDescriptor, data, size, 0);
    }
}

void Remote::onWiFiConnected()
{
    sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = 0xFFFFFFFF;
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(12345);

    mDescriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (mDescriptor < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        mDescriptor = -1;
        return;
    }
    connect(mDescriptor, (sockaddr *) &destAddr, sizeof(destAddr));
}

void Remote::onWiFiDisconnected()
{
    if (mDescriptor != -1) {
        close(mDescriptor);
    }
}

