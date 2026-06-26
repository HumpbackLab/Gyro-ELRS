#include "gyro_sc7u22.h"

#if defined(HAS_GYRO)

#include <Arduino.h>
#include <Wire.h>
#include "logging.h"

uint8_t Gyro_SC7U22::m_i2cAddress = 0;

Gyro_SC7U22::Gyro_SC7U22()
{
}

void Gyro_SC7U22::writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(m_i2cAddress);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void Gyro_SC7U22::writeRegisterDelay(uint8_t reg, uint8_t value, unsigned delayMs)
{
    writeRegister(reg, value);
    if (delayMs)
    {
        delay(delayMs);
    }
}

bool Gyro_SC7U22::readRegister(uint8_t reg, uint8_t *value, bool stop)
{
    Wire.beginTransmission(m_i2cAddress);
    Wire.write(reg);
    // endTransmission(true) = send STOP; endTransmission(false) = repeated start
    if (Wire.endTransmission(stop) != 0)
    {
        return false;
    }

    if (Wire.requestFrom(m_i2cAddress, (uint8_t)1) != 1)
    {
        return false;
    }

    *value = Wire.read();
    return true;
}

bool Gyro_SC7U22::readBurst(uint8_t reg, uint8_t *data, size_t size)
{
    Wire.beginTransmission(m_i2cAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
    {
        return false;
    }

    if (Wire.requestFrom(m_i2cAddress, size) != size)
    {
        return false;
    }

    Wire.readBytes(data, size);
    return true;
}

bool Gyro_SC7U22::detect()
{
    uint8_t chipId = 0;

    // Try SDO_HIGH address (0x19) first, then SDO_LOW (0x18)
    // SDO floating or pulled HIGH → 0x19 (recommended per datasheet)
    // SDO pulled LOW → 0x18 (requires disabling internal pull-up via DIG_CTRL for power efficiency)
    for (uint8_t addrIdx = 0; addrIdx < 2; addrIdx++)
    {
        m_i2cAddress = (addrIdx == 0) ? I2C_ADDRESS_SDO_HIGH : I2C_ADDRESS_SDO_LOW;

        // After power-up SEG_SEL defaults to 0x00 (general register bank accessible).
        // Still write 0x00 to ensure we're in segment 0 (chip may have been left in
        // a different segment by a prior incomplete transaction).
        Wire.beginTransmission(m_i2cAddress);
        Wire.write(REG_SEG_SEL);
        Wire.write(0x00);
        uint8_t segErr = Wire.endTransmission();
        delay(1);

        if (segErr != 0)
        {
            GyroDiagPrintf("No ACK at 0x%02X (err=%u)", m_i2cAddress, segErr);
            continue; // Skip read attempts, device not at this address
        }

        // Attempt to read chip ID up to 5 times
        // Use repeated start (stop=false) per datasheet I2C read protocol
        for (int attempts = 0; attempts < 5; attempts++)
        {
            if (readRegister(REG_WHO_AM_I, &chipId, false))
            {
                if (chipId == CHIP_ID)
                {
                    GyroDiagPrintf("Detected SC7U22 at 0x%02X", m_i2cAddress);
                    return true;
                }
                GyroDiagPrintf("Bad WHO_AM_I at 0x%02X: 0x%02X (exp 0x%02X)",
                      m_i2cAddress, chipId, CHIP_ID);
            }
            delay(10);
        }
        GyroDiagPrintf("No response at 0x%02X (5 tries)", m_i2cAddress);
    }

    GyroDiagPrintf("Detection failed. Check: 1) CSB(pin12)=HIGH 2) I2C pull-ups 3) Wiring");
    return false;
}

void Gyro_SC7U22::initialize()
{
    // Full initialization sequence from the SC7U22 datasheet / INAV driver

    // 1. Select segment 0
    writeRegisterDelay(REG_SEG_SEL, 0x00, 1);

    // 2. Configure communication: Block Data Update + Auto-increment
    writeRegisterDelay(REG_COM_CONF, COM_CONF_BDU | COM_CONF_ADDR_AUTO, 1);

    // 3. Double software reset (required for reliable startup)
    writeRegisterDelay(REG_SOFT_RST, SOFT_RESET_VALUE, 1);
    writeRegisterDelay(REG_SOFT_RST, SOFT_RESET_VALUE, RESET_DELAY_MS);

    // 4. Re-select segment 0 after reset
    writeRegisterDelay(REG_SEG_SEL, 0x00, 1);

    // 5. Re-configure communication
    writeRegisterDelay(REG_COM_CONF, COM_CONF_BDU | COM_CONF_ADDR_AUTO, 1);

    // 6. Power off all sensors before configuration
    writeRegisterDelay(REG_PWR_CTRL, 0x00, 1);

    // 7. Set accelerometer range: +/- 16G
    writeRegisterDelay(REG_ACC_RANGE, ACC_RANGE_16G, 1);

    // 8. Set gyroscope range: +/- 2000 DPS
    writeRegisterDelay(REG_GYR_RANGE, GYR_RANGE_2000DPS, 1);

    // 9. Configure accelerometer: high performance, OSR4, ODR 1600Hz
    writeRegisterDelay(REG_ACC_CONF, ACC_FILTER_PERF | ACC_BWP_OSR4_AVG1 | ACC_ODR_1600, 1);

    // 10. Configure gyroscope: high performance, OSR4, ODR 1600Hz
    writeRegisterDelay(REG_GYR_CONF, GYR_CONF_VALUE, CONFIG_SETTLE_DELAY_MS);

    // 11. Power on: temperature + accelerometer + gyroscope
    writeRegisterDelay(REG_PWR_CTRL, PWR_CTRL_TEMP_EN | PWR_CTRL_ACC_EN | PWR_CTRL_GYR_EN,
                       SENSOR_START_DELAY_MS);

    m_initialized = true;
    GyroDiagPrintf("SC7U22 init OK (1600Hz ODR, ±2000dps)");
}

uint8_t Gyro_SC7U22::getGyroDuration()
{
    // 250Hz gyro poll rate (4ms). I2C read takes ~400us at 400kHz,
    // leaving ~3.6ms per cycle for WiFi, PWM, and FC computation.
    return 4;
}

void Gyro_SC7U22::startGyro()
{
    // Data is always ready at 1600Hz ODR. No start trigger needed.
    // The read is done in getGyroDps().
}

bool Gyro_SC7U22::getGyroDps(GyroVector3 &gyro)
{
    GyroVector3 accelMps2;
    bool accelValid = false;
    return getMotion(gyro, accelMps2, accelValid);
}

bool Gyro_SC7U22::getMotion(GyroVector3 &gyro, GyroVector3 &accelMps2, bool &accelValid)
{
    // Read 12 bytes starting from ACC_XH (0x0C)
    // Data layout: Accel X[2], Accel Y[2], Accel Z[2], Gyro X[2], Gyro Y[2], Gyro Z[2]
    // All values are 16-bit signed big-endian
    uint8_t raw[12];

    if (!readBurst(REG_ACC_XH, raw, sizeof(raw)))
    {
        return false;
    }

    int16_t accelRawX = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    int16_t accelRawY = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    int16_t accelRawZ = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);

    // Extract gyro data from bytes 6-11
    int16_t gyroRawX = (int16_t)(((uint16_t)raw[6] << 8) | raw[7]);
    int16_t gyroRawY = (int16_t)(((uint16_t)raw[8] << 8) | raw[9]);
    int16_t gyroRawZ = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);

    accelMps2.x = (float)accelRawX * ACCEL_SCALE_MPS2;
    accelMps2.y = (float)accelRawY * ACCEL_SCALE_MPS2;
    accelMps2.z = (float)accelRawZ * ACCEL_SCALE_MPS2;
    accelValid = true;

    // Convert to degrees per second
    gyro.x = (float)gyroRawX * GYRO_SCALE;
    gyro.y = (float)gyroRawY * GYRO_SCALE;
    gyro.z = (float)gyroRawZ * GYRO_SCALE;

    return true;
}

#endif // HAS_GYRO
