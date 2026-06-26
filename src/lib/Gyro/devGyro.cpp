#include "devGyro.h"

#if defined(HAS_GYRO)

#include <Arduino.h>
#include "logging.h"
#include "gyro_sc7u22.h"

#define GYRO_STARTUP_INTERVAL 100

static GyroBase *gyro;
static eGyroReadState GyroReadState;
static GyroSample latestSample = {};
static bool latestSampleValid = false;
static String gyroDiagMsg;

extern bool i2c_enabled;

void GyroDiagPrintf(const char *fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (gyroDiagMsg.length() > 0) gyroDiagMsg += '\n';
    gyroDiagMsg += buf;
}

const char *GyroGetDiagMsg()
{
    return gyroDiagMsg.c_str();
}

static bool Gyro_Detect()
{
#if defined(USE_I2C)
    if (i2c_enabled)
    {
        if (Gyro_SC7U22::detect())
        {
            gyro = new Gyro_SC7U22();
            return true;
        }
        // Additional I2C gyro backends can be added here.
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

    GyroDiagPrintf("Init failed, retrying...");
    return GYRO_STARTUP_INTERVAL;
}

static void Gyro_Publish(const GyroVector3 &gyroDps, const GyroVector3 &accelMps2, bool accelValid)
{
    latestSample.gyroDps = gyroDps;
    latestSample.accelMps2 = accelMps2;
    latestSample.accelValid = accelValid;
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
                GyroVector3 accelMps2;
                bool accelValid = false;
                if (!gyro->getMotion(gyroDps, accelMps2, accelValid))
                {
                    return DURATION_IMMEDIATELY;
                }
                Gyro_Publish(gyroDps, accelMps2, accelValid);
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
