#include "devFlightControl.h"

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

#include "FlightControl.h"
#include "common.h"
#include "devServoOutput.h"
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

static int timeout()
{
    runtime.update(micros());
    if (runtime.ready())
    {
        // Trigger servo update after flight control produces new mixer output,
        // so PWM works even without RC packets (e.g. WiFi debug mode).
        servoNewChannelsAvailable();
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
