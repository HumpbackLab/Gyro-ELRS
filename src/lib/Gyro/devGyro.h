#pragma once

#include "common.h"
#include "device.h"
#include "gyro_base.h"

#if defined(TARGET_UNIFIED_RX)
    #define HAS_GYRO
#endif

#if defined(HAS_GYRO)

enum eGyroReadState : uint8_t
{
    grsNoGyro,
    grsUninitialized,
    grsReadGyro,
    grsWaitingGyro
};

struct GyroSample
{
    GyroVector3 gyroDps;
    GyroVector3 accelMps2;
    uint32_t timestampMs;
    bool accelValid;
};

extern device_t Gyro_device;

bool GyroIsInitialized();
bool GyroGetSample(GyroSample &sample);

// Diagnostic message buffer for web UI status page
void GyroDiagPrintf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
const char *GyroGetDiagMsg();

#endif
