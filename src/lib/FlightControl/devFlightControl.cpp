#include "devFlightControl.h"

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

#include "FlightControl.h"
#include "common.h"
#include <Arduino.h>

static FlightControlRuntime runtime;

void ICACHE_RAM_ATTR flightControlNewChannelsAvailable()
{
    runtime.markChannelsAvailable();
}

const FlightControlAttitude &flightControlGetAttitude()
{
    return runtime.attitude();
}

const FlightControlMixerOutput &flightControlGetMixerOutput()
{
    return runtime.mixerOutput();
}

static void initialize()
{
    runtime.begin();
}

static int event()
{
    if (!runtime.ready() || connectionState != connected)
    {
        runtime.reset();
        return DURATION_NEVER;
    }
    return DURATION_IMMEDIATELY;
}

static int timeout()
{
    runtime.update(micros());
    return DURATION_IMMEDIATELY;
}

device_t FlightControl_device = {
    .initialize = initialize,
    .start = nullptr,
    .event = event,
    .timeout = timeout,
};

#endif
