#include "devFlightControl.h"

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

#include "FlightControl.h"
#include "FlightControlConfig.h"
#include "common.h"
#include "crsf_protocol.h"
#if defined(USE_MSP_WIFI)
#include "msptypes.h"
#include "tcpsocket.h"
#endif
#include <Arduino.h>
#include <math.h>

static FlightControlRuntime runtime;
static constexpr int FC_READY_RETRY_INTERVAL_MS = 100;

#if defined(USE_MSP_WIFI)
extern TCPSOCKET wifi2tcp;
#endif

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

#if defined(USE_MSP_WIFI)
static void appendU16(uint8_t *payload, uint16_t &offset, uint16_t value)
{
    payload[offset++] = value & 0xFF;
    payload[offset++] = value >> 8;
}

static void appendI16(uint8_t *payload, uint16_t &offset, int16_t value)
{
    appendU16(payload, offset, (uint16_t)value);
}

static int16_t scaledFloatToI16(float value, float scale)
{
    return (int16_t)constrain((int32_t)lroundf(value * scale), -32768, 32767);
}

static bool handleLocalMspRequest(uint16_t function,
    const uint8_t *requestPayload, uint16_t requestPayloadLen,
    uint8_t *responsePayload, uint16_t responseCapacity,
    uint16_t &responsePayloadLen)
{
    (void)requestPayload;
    (void)requestPayloadLen;

    if (function != MSP_ELRS_FC_DEBUG || responseCapacity < 10)
    {
        return false;
    }

    FlightControlDebugSnapshot snapshot = {};
    flightControlGetDebugSnapshot(snapshot);

    responsePayloadLen = 0;
    appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.attitude.rollDeg, 100.0f));
    appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.attitude.pitchDeg, 100.0f));
    appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.attitude.yawDeg, 100.0f));
    appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.accelAttitude.rollDeg, 100.0f));
    appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.accelAttitude.pitchDeg, 100.0f));
    return true;
}
#endif

static void initialize()
{
    flightControlConfig.Load();
    runtime.begin();
#if defined(USE_MSP_WIFI)
    wifi2tcp.setLocalMspHandler(handleLocalMspRequest);
#endif
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
