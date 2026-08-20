#include "devFlightControl.h"

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

#include "FlightControl.h"
#include "FlightControlConfig.h"
#include "CRSF.h"
#include "common.h"
#include "crsf_protocol.h"
#include "telemetry.h"
#include "devWIFI.h"
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
static bool wifiSwitchCandidate = false;
static bool wifiSwitchCoexist = false;
static uint32_t wifiSwitchSinceMs = 0;

static void updateWifiModeSwitch(uint32_t nowMs)
{
    // RF is the safe fallback; coexistence is the only configurable condition.
    const uint8_t channel = flightControlConfig.GetWifiCoexistChannel();
    const bool candidate = flightControlConfig.GetWifiCoexistEnabled() &&
        FlightControlRangeIsActive(flightControlConfig.GetWifiCoexistRange(), CRSF_to_US(ChannelData[channel]));
    // Stabilize the selected condition for 300 ms before acting.
    if (candidate != wifiSwitchCandidate) {
        wifiSwitchCandidate = candidate;
        wifiSwitchSinceMs = nowMs;
        return;
    }
    if (candidate == wifiSwitchCoexist || (uint32_t)(nowMs - wifiSwitchSinceMs) < 300) return;
    wifiSwitchCoexist = candidate;
    setFlightControlWifiCoexist(candidate);
}

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

static void appendU32(uint8_t *payload, uint16_t &offset, uint32_t value)
{
    appendU16(payload, offset, value & 0xFFFF);
    appendU16(payload, offset, value >> 16);
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

    static constexpr uint16_t FC_DEBUG_PAYLOAD_SIZE = 40;
    static constexpr uint16_t FC_PID_DEBUG_PAYLOAD_SIZE = 28;
    static constexpr uint16_t FC_IMU_DEBUG_PAYLOAD_SIZE = 18;
    static constexpr uint16_t FC_ATTITUDE_DEBUG_PAYLOAD_SIZE = 6;
    // Keep the original combined response wire-compatible with released
    // configurator v0.1.7; new clients use the split PID and IMU requests.
    const bool legacyDebug = function == MSP_ELRS_FC_DEBUG;
    const bool pidDebug = function == MSP_ELRS_FC_PID_DEBUG;
    const bool imuDebug = function == MSP_ELRS_FC_IMU_DEBUG;
    const bool attitudeDebug = function == MSP_ELRS_FC_ATTITUDE_DEBUG;
    const uint16_t requiredCapacity = legacyDebug ? FC_DEBUG_PAYLOAD_SIZE
        : pidDebug ? FC_PID_DEBUG_PAYLOAD_SIZE
        : imuDebug ? FC_IMU_DEBUG_PAYLOAD_SIZE
        : attitudeDebug ? FC_ATTITUDE_DEBUG_PAYLOAD_SIZE : 0;
    if (requiredCapacity == 0 || responseCapacity < requiredCapacity)
    {
        return false;
    }

    FlightControlDebugSnapshot snapshot = {};
    flightControlGetDebugSnapshot(snapshot);

    responsePayloadLen = 0;
    if (legacyDebug || imuDebug || attitudeDebug)
    {
        appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.attitude.rollDeg, 100.0f));
        appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.attitude.pitchDeg, 100.0f));
        appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.attitude.yawDeg, 100.0f));
        if (attitudeDebug) return true;
        if (legacyDebug)
        {
            appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.accelAttitude.rollDeg, 100.0f));
            appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.accelAttitude.pitchDeg, 100.0f));
        }
        appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.imu.gyroDps.x, 100.0f));
        appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.imu.gyroDps.y, 100.0f));
        appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.imu.gyroDps.z, 100.0f));
        appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.imu.accelMps2.x, 1000.0f));
        appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.imu.accelMps2.y, 1000.0f));
        appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.imu.accelMps2.z, 1000.0f));
        if (imuDebug) return true;
    }

    responsePayload[responsePayloadLen++] = (uint8_t)snapshot.mode;
    responsePayload[responsePayloadLen++] = snapshot.armed ? 1 : 0;
    appendU32(responsePayload, responsePayloadLen, snapshot.updateTimestampMs);
    appendU16(responsePayload, responsePayloadLen, snapshot.updateDtUs);
    appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.rollAngleTarget, 100.0f));
    appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.pitchAngleTarget, 100.0f));
    if (pidDebug)
    {
        appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.rollAngleState, 100.0f));
        appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.pitchAngleState, 100.0f));
    }
    appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.rollRateTarget, 10.0f));
    appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.pitchRateTarget, 10.0f));
    appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.yawRateTarget, 10.0f));
    if (pidDebug)
    {
        // PID charts report the gyro signal consumed by the controller after
        // the configured low-pass stage.
        appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.filteredGyroDps.x, 100.0f));
        appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.filteredGyroDps.y, 100.0f));
        appendI16(responsePayload, responsePayloadLen, scaledFloatToI16(snapshot.filteredGyroDps.z, 100.0f));
    }
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
    // Start or resume on the same 4 ms cadence used by timeout(). An immediate
    // update here would create a short-dt outlier whenever connection state or
    // model-match events fire.
    return 4;
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
    for (uint8_t mode = FLIGHT_CONTROL_MODE_RATE; mode < FLIGHT_CONTROL_MODE_COUNT; ++mode)
    {
        if (!flightControlConfig.GetModeEnabled((FlightControlMode)mode))
        {
            continue;
        }
        const uint8_t modeChannel = flightControlConfig.GetModeChannel((FlightControlMode)mode);
        if (modeChannel < FC_MODE_CHANNEL_MIN || modeChannel > FC_MODE_CHANNEL_MAX ||
            ChannelData[modeChannel] < CRSF_CHANNEL_VALUE_MIN || ChannelData[modeChannel] > CRSF_CHANNEL_VALUE_MAX)
        {
            return false;
        }
    }
    if (flightControlConfig.GetArmMode())
    {
        const uint8_t armChannel = flightControlConfig.GetArmChannel();
        if (armChannel < FC_MODE_CHANNEL_MIN || armChannel > FC_MODE_CHANNEL_MAX ||
            ChannelData[armChannel] < CRSF_CHANNEL_VALUE_MIN || ChannelData[armChannel] > CRSF_CHANNEL_VALUE_MAX)
        {
            return false;
        }
    }
    return true;
}

static int timeout()
{
    // Always inspect the configured mode channel while the receiver is connected.
    if (connectionState == connected && connectionHasModelMatch && teamraceHasModelMatch)
    {
        updateWifiModeSwitch(millis());
    }

    if (!rcInputReady())
    {
        if (connectionState == wifiUpdate)
        {
            runtime.updateAttitudeOnly(micros(), false);
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
        runtime.updateAttitudeOnly(nowUs, true);
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
