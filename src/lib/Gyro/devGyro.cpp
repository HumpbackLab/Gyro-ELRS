#include "devGyro.h"

#if defined(HAS_GYRO)

#include <Arduino.h>
#include "logging.h"

#define GYRO_STARTUP_INTERVAL 100

static GyroBase *gyro;
static eGyroReadState GyroReadState;
static GyroSample latestSample = {};
static bool latestSampleValid = false;

extern bool i2c_enabled;

static bool Gyro_Detect()
{
#if defined(USE_I2C)
    if (i2c_enabled)
    {
        // Future I2C gyro backends are detected here.
    }
#endif
    return false;
}

static int Gyro_Init()
{
    gyro->initialize();
    if (gyro->isInitialized())
    {
        latestSampleValid = false;
        GyroReadState = grsReadGyro;
        return DURATION_IMMEDIATELY;
    }

    return GYRO_STARTUP_INTERVAL;
}

static void Gyro_Publish(const GyroVector3 &gyroDps)
{
    latestSample.gyroDps = gyroDps;
    latestSample.timestampMs = millis();
    latestSampleValid = true;
}

bool GyroIsInitialized()
{
    return gyro != nullptr && gyro->isInitialized();
}

bool GyroGetSample(GyroSample &sample)
{
    if (!latestSampleValid)
    {
        return false;
    }

    sample = latestSample;
    return true;
}

static int start()
{
    gyro = nullptr;
    latestSampleValid = false;

    if (Gyro_Detect())
    {
        GyroReadState = grsUninitialized;
        return GYRO_STARTUP_INTERVAL;
    }

    DBGLN("No gyro detected");
    GyroReadState = grsNoGyro;
    return DURATION_NEVER;
}

static int timeout()
{
    if (connectionState >= MODE_STATES)
    {
        return DURATION_NEVER;
    }

    switch (GyroReadState)
    {
        default:
        case grsNoGyro:
            return DURATION_NEVER;

        case grsUninitialized:
            return Gyro_Init();

        case grsReadGyro:
            {
                const uint8_t gyroDuration = gyro->getGyroDuration();
                GyroReadState = grsWaitingGyro;
                if (gyroDuration != 0)
                {
                    gyro->startGyro();
                    return gyroDuration;
                }
            }
            // fallthrough

        case grsWaitingGyro:
            {
                GyroVector3 gyroDps;
                if (!gyro->getGyroDps(gyroDps))
                {
                    return DURATION_IMMEDIATELY;
                }
                Gyro_Publish(gyroDps);
                GyroReadState = grsReadGyro;
                return DURATION_IMMEDIATELY;
            }
    }
}

device_t Gyro_device = {
    .initialize = nullptr,
    .start = start,
    .event = nullptr,
    .timeout = timeout,
};

#endif
