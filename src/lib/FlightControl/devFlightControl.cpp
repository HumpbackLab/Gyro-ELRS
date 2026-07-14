#include "devFlightControl.h"

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

#include "FlightControl.h"
#include "FlightControlConfig.h"
#include "CRSF.h"
#include "common.h"
#include "crsf_protocol.h"
#include "telemetry.h"
#if defined(USE_MSP_WIFI)
#include "msptypes.h"
#include "tcpsocket.h"
#endif
#include <Arduino.h>
#include <math.h>

static FlightControlRuntime runtime;
static constexpr int FC_READY_RETRY_INTERVAL_MS = 100;
static constexpr uint32_t FC_ATTITUDE_REPORT_INTERVAL_MS = 100;
static constexpr float FC_DEGREES_TO_CRSF_ATTITUDE = PI / 180.0f * 10000.0f;
static uint32_t lastAttitudeReportMs;

extern Telemetry telemetry;
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

static void reportAttitude(uint32_t nowMs)
{
    if ((uint32_t)(nowMs - lastAttitudeReportMs) < FC_ATTITUDE_REPORT_INTERVAL_MS)
    {
        return;
    }

    FlightControlDebugSnapshot snapshot = {};
    if (!runtime.getDebugSnapshot(snapshot) || !snapshot.attitudeValid)
    {
        return;
    }

    CRSF_MK_FRAME_T(crsf_sensor_attitude_t) crsfAttitude = {0};
    crsfAttitude.p.pitch = htobe16((int16_t)lroundf(snapshot.attitude.pitchDeg * FC_DEGREES_TO_CRSF_ATTITUDE));
    crsfAttitude.p.roll = htobe16((int16_t)lroundf(snapshot.attitude.rollDeg * FC_DEGREES_TO_CRSF_ATTITUDE));
    crsfAttitude.p.yaw = 0;
    CRSF::SetHeaderAndCrc((uint8_t *)&crsfAttitude, CRSF_FRAMETYPE_ATTITUDE,
        CRSF_FRAME_SIZE(sizeof(crsf_sensor_attitude_t)), CRSF_ADDRESS_CRSF_TRANSMITTER);
    telemetry.AppendTelemetryPackage((uint8_t *)&crsfAttitude);
    lastAttitudeReportMs = nowMs;
}

static void initialize()
{
    flightControlConfig.Load();
    runtime.begin();
    lastAttitudeReportMs = 0;
#if defined(USE_MSP_WIFI)
    wifi2tcp.setLocalMspHandler(handleLocalMspRequest);
#endif
}

static int event()
{
    runtime.refreshReadyState();
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

    const uint32_t nowUs = micros();
    if (runtime.ready())
    {
        runtime.update(nowUs);
    }
    else
    {
        // Attitude estimation only needs the IMU. Keep it running even when
        // PID or mixer configuration is incomplete.
        runtime.updateAttitudeOnly(nowUs);
    }

    reportAttitude(millis());
    return 4; // 250Hz, matched to FC_UPDATE_INTERVAL_US
}

device_t FlightControl_device = {
    .initialize = initialize,
    .start = nullptr,
    .event = event,
    .timeout = timeout,
};

#endif
