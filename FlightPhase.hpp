#pragma once

struct FlightPhase
{
    virtual void process(Flyer * flyer) = 0;
};
