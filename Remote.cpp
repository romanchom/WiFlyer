#include "Remote.hpp"

#include "WiFi.hpp"

#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>

#include <esp_log.h>
#include <iostream>

char const * TAG = "Remote";

Remote::Remote() :
    mDescriptor(-1)
{}

void Remote::write(std::byte const * data, size_t size)
{
    sockaddr_in destAddr;
    destAddr.sin_family = AF_INET;
    destAddr.sin_addr.s_addr = 0xFFFFFFFF;
    destAddr.sin_port = htons(12345);

    if (mDescriptor != -1) {
        sendto(mDescriptor, data, size, 0, (sockaddr *) &destAddr, sizeof(destAddr));
    }
}

size_t Remote::read(std::byte * data, size_t size)
{
    auto ret = recv(mDescriptor, data, size, 0);
    if (ret < 0) {
        ret = 0;
    }
    return ret;
}

void Remote::onWiFiConnected()
{
    mDescriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (mDescriptor < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        mDescriptor = -1;
        return;
    }
    fcntl(mDescriptor, F_SETFL, fcntl(mDescriptor, F_GETFL, 0) | O_NONBLOCK);

    sockaddr_in destAddr;
    destAddr.sin_family = AF_INET;
    destAddr.sin_addr.s_addr = 0;
    destAddr.sin_port = htons(12346);
    auto ret = bind(mDescriptor, (sockaddr *) &destAddr, sizeof(destAddr));
    if (ret != 0) {
        std::cout << "Bind failed: " << errno << std::endl;
    }
}

void Remote::onWiFiDisconnected()
{
    if (mDescriptor != -1) {
        close(mDescriptor);
    }
}

