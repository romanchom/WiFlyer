#pragma once

struct PIDMessage
{
    enum { id = 0 };
    float rollP;
    float rollI;
    float rollD;
    float pitchP;
    float pitchI;
    float pitchD;
};

struct SteeringMessage
{
    enum { id = 1 };
    int enabled;
    float throttle;
    int reset;
};
