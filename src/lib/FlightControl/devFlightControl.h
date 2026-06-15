#pragma once

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

#include "device.h"
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

const FlightControlAttitude &flightControlGetAttitude();
const FlightControlMixerOutput &flightControlGetMixerOutput();

#else
#endif
