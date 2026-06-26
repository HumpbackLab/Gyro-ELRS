#pragma once

#include <stdint.h>

struct GyroVector3
{
    float x;
    float y;
    float z;
};

class GyroBase
{
public:
    GyroBase() : m_initialized(false) {}

    virtual void initialize() = 0;
    virtual uint8_t getGyroDuration() = 0;
    virtual void startGyro() = 0;
    virtual bool getGyroDps(GyroVector3 &gyro) = 0;
    virtual bool getMotion(GyroVector3 &gyro, GyroVector3 &accelMps2, bool &accelValid)
    {
        accelMps2 = {};
        accelValid = false;
        return getGyroDps(gyro);
    }

    bool isInitialized() const { return m_initialized; }

protected:
    bool m_initialized;
};
