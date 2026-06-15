#include "devFlightControl.h"

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

#include "FlightControl.h"
#include "common.h"
#include "devServoOutput.h"
#include <Arduino.h>

static FlightControlRuntime runtime;

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
    if (!runtime.ready())
    {
        runtime.reset();
        return DURATION_NEVER;
    }
    return DURATION_IMMEDIATELY;
}

static int timeout()
{
    runtime.update(micros());
    // Trigger servo update after flight control produces new mixer output,
    // so PWM works even without RC packets (e.g. WiFi debug mode).
    servoNewChannelsAvailable();
    return 4; // 250Hz, matched to FC_UPDATE_INTERVAL_US
}

device_t FlightControl_device = {
    .initialize = initialize,
    .start = nullptr,
    .event = event,
    .timeout = timeout,
};

#endif
