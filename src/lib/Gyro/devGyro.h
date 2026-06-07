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
    uint32_t timestampMs;
};

extern device_t Gyro_device;

bool GyroIsInitialized();
bool GyroGetSample(GyroSample &sample);

#endif
