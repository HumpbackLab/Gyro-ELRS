#pragma once

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

#include "device.h"
#include "FlightControlConfig.h"
#include <stdint.h>

#define FLIGHT_CONTROL_MAX_MOTORS 8

extern device_t FlightControl_device;

struct FlightControlVector3 {
    float x;
    float y;
    float z;
};

struct FlightControlImuSample {
    FlightControlVector3 gyroDps;
    FlightControlVector3 accelMps2;
    uint32_t timestampMs;
    bool gyroValid;
    bool accelValid;
};

struct FlightControlAttitude {
    float rollDeg;
    float pitchDeg;
    float yawDeg;
};

struct FlightControlMixerOutput {
    uint8_t motorCount;
    uint16_t motorUs[FLIGHT_CONTROL_MAX_MOTORS];
};

struct FlightControlDebugSnapshot {
    FlightControlImuSample imu;
    FlightControlVector3 filteredGyroDps;
    FlightControlAttitude attitude;
    FlightControlAttitude accelAttitude;
    FlightControlMixerOutput mixerOutput;
    uint32_t updateTimestampMs;
    uint16_t updateDtUs;
    uint16_t sampleAgeMs;
    uint16_t complementaryAlphaPermille;
    bool sensorsReady;
    bool mixerReady;
    bool pidReady;
    FlightControlMode mode;
    bool attitudeValid;
    float rollAngleTarget;
    float pitchAngleTarget;
    float rollAngleState;
    float pitchAngleState;
    float rollRateTarget;
    float pitchRateTarget;
    float yawRateTarget;
    bool armed;
};

const FlightControlAttitude &flightControlGetAttitude();
const FlightControlMixerOutput &flightControlGetMixerOutput();
bool flightControlGetDebugSnapshot(FlightControlDebugSnapshot &snapshot);

#else
#endif
