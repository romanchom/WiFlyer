#pragma once

#include <rom/ets_sys.h>

struct EspWait
{
    static void microseconds(int count)
    {
        ets_delay_us(count);
    }
};