#include "devFlightControl.h"

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

#include "FlightControl.h"
#include "common.h"
#include "crsf_protocol.h"
#include <Arduino.h>

static FlightControlRuntime runtime;
static constexpr int FC_READY_RETRY_INTERVAL_MS = 100;

const FlightControlAttitude &flightControlGetAttitude()
{
    return runtime.attitude();
}

const FlightControlMixerOutput &flightControlGetMixerOutput()
{
    return runtime.mixerOutput();
}

bool flightControlGetDebugSnapshot(FlightControlDebugSnapshot &snapshot)
{
    return runtime.getDebugSnapshot(snapshot);
}

static void initialize()
{
    runtime.begin();
}

static int event()
{
    if (!runtime.refreshReadyState())
    {
        runtime.reset();
        return FC_READY_RETRY_INTERVAL_MS;
    }
    return DURATION_IMMEDIATELY;
}

static bool rcInputReady()
{
    if (connectionState != connected || !connectionHasModelMatch || !teamraceHasModelMatch)
    {
        return false;
    }

    for (uint8_t i = 0; i < 4; ++i)
    {
        if (ChannelData[i] < CRSF_CHANNEL_VALUE_MIN || ChannelData[i] > CRSF_CHANNEL_VALUE_MAX)
        {
            return false;
        }
    }
    return true;
}

static int timeout()
{
    if (!rcInputReady())
    {
        if (connectionState == wifiUpdate)
        {
            runtime.updateAttitudeOnly(micros());
            return 4;
        }
        runtime.reset();
        return FC_READY_RETRY_INTERVAL_MS;
    }

    runtime.update(micros());
    if (runtime.ready())
    {
        return 4; // 250Hz, matched to FC_UPDATE_INTERVAL_US
    }
    return FC_READY_RETRY_INTERVAL_MS;
}

device_t FlightControl_device = {
    .initialize = initialize,
    .start = nullptr,
    .event = event,
    .timeout = timeout,
};

#endif
