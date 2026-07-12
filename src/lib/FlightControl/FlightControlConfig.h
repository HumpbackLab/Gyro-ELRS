#pragma once

#include "targets.h"

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

#include <stdint.h>

constexpr uint8_t FC_PID_TERM_COUNT = 12;
constexpr uint8_t FC_MIXER_COLUMNS = 4;
constexpr uint8_t FC_MIXER_MAX_MOTORS = 8;
constexpr uint8_t FC_MIXER_VALUE_COUNT = FC_MIXER_COLUMNS * FC_MIXER_MAX_MOTORS;
constexpr uint8_t FC_ORIENTATION_VALUE_COUNT = 9;

class FlightControlConfig
{
public:
    void Load();
    bool Commit();
    void SetDefaults();

    const int16_t *GetRatePid() const { return m_ratePid; }
    const int16_t *GetAnglePid() const { return m_anglePid; }
    bool GetAngleMode() const { return m_angleMode; }
    bool GetArmMode() const { return m_armMode; }
    const float *GetMixer() const { return m_mixer; }
    uint8_t GetMixerCount() const { return m_mixerCount; }
    const float *GetOrientation() const { return m_orientation; }
    bool IsModified() const { return m_modified; }

    void SetRatePid(uint8_t index, int16_t value);
    void SetAnglePid(uint8_t index, int16_t value);
    void SetAngleMode(bool enabled);
    void SetArmMode(bool enabled);
    void SetMixer(const float *values, uint8_t count);
    void SetOrientation(const float *values, uint8_t count);

private:
    int16_t m_ratePid[FC_PID_TERM_COUNT] = {};
    int16_t m_anglePid[FC_PID_TERM_COUNT] = {};
    float m_mixer[FC_MIXER_VALUE_COUNT] = {};
    float m_orientation[FC_ORIENTATION_VALUE_COUNT] = {};
    uint8_t m_mixerCount = 0;
    bool m_angleMode = false;
    bool m_armMode = false;
    bool m_modified = false;
};

extern FlightControlConfig flightControlConfig;

#endif
