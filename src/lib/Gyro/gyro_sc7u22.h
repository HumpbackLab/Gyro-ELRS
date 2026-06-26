#pragma once

#include "devGyro.h"
#include "gyro_base.h"
#include <stdint.h>

#if defined(HAS_GYRO)

class Gyro_SC7U22 : public GyroBase
{
public:
    Gyro_SC7U22();

    void initialize() override;
    uint8_t getGyroDuration() override;
    void startGyro() override;
    bool getGyroDps(GyroVector3 &gyro) override;
    bool getMotion(GyroVector3 &gyro, GyroVector3 &accelMps2, bool &accelValid) override;

    static bool detect();

private:
    static const uint8_t I2C_ADDRESS_SDO_LOW  = 0x18;
    static const uint8_t I2C_ADDRESS_SDO_HIGH = 0x19;

    // Register addresses
    static const uint8_t REG_WHO_AM_I  = 0x01;
    static const uint8_t REG_COM_CONF  = 0x04;
    static const uint8_t REG_ACC_XH    = 0x0C;
    static const uint8_t REG_ACC_CONF  = 0x40;
    static const uint8_t REG_ACC_RANGE = 0x41;
    static const uint8_t REG_GYR_CONF  = 0x42;
    static const uint8_t REG_GYR_RANGE = 0x43;
    static const uint8_t REG_SOFT_RST  = 0x4A;
    static const uint8_t REG_PWR_CTRL  = 0x7D;
    static const uint8_t REG_SEG_SEL   = 0x7F;

    // Bit definitions
    static const uint8_t COM_CONF_BDU       = 0x40; // BIT(6) - Block Data Update
    static const uint8_t COM_CONF_ADDR_AUTO = 0x10; // BIT(4) - Auto-increment address

    static const uint8_t PWR_CTRL_TEMP_EN = 0x08; // BIT(3)
    static const uint8_t PWR_CTRL_ACC_EN  = 0x04; // BIT(2)
    static const uint8_t PWR_CTRL_GYR_EN  = 0x02; // BIT(1)

    static const uint8_t ACC_FILTER_PERF     = 0x80; // BIT(7)
    static const uint8_t ACC_BWP_OSR4_AVG1  = 0x00; // 0x00 << 4
    static const uint8_t ACC_ODR_1600        = 0x0C;

    static const uint8_t ACC_RANGE_16G       = 0x03;

    static const uint8_t GYR_FILTER_PERF     = 0x80; // BIT(7)
    static const uint8_t GYR_BWP_OSR4_AVG1  = 0x00; // 0x00 << 4
    static const uint8_t GYR_RANGE_2000DPS   = 0x00;

    // GYR_CONF value: PERF | OSR4_AVG1 | ODR_1600 (0x0C)
    static const uint8_t GYR_CONF_VALUE = 0x80 | 0x00 | 0x0C;

    static const uint8_t CHIP_ID           = 0x6A;
    static const uint8_t SOFT_RESET_VALUE  = 0xA5;
    static const uint8_t RESET_DELAY_MS    = 200;
    static const uint8_t SENSOR_START_DELAY_MS = 60;
    static const uint8_t CONFIG_SETTLE_DELAY_MS = 2;

    // Scale factor for 2000 DPS range: 2000 / 32768 = 1 / 16.384
    static constexpr float GYRO_SCALE = 1.0f / 16.384f;
    // Scale factor for 16G range: 16G / 32768, converted to m/s^2.
    static constexpr float ACCEL_SCALE_MPS2 = (16.0f * 9.80665f) / 32768.0f;

    static uint8_t m_i2cAddress;

    static void writeRegister(uint8_t reg, uint8_t value);
    static void writeRegisterDelay(uint8_t reg, uint8_t value, unsigned delayMs);
    static bool readRegister(uint8_t reg, uint8_t *value, bool stop = true);
    static bool readBurst(uint8_t reg, uint8_t *data, size_t size);
};

#endif // HAS_GYRO
