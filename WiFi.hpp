#pragma once

#include <functional>

void initWiFi(std::function<void()> && onGotIP);
