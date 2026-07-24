#pragma once

#include "targets.h"

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

#include <stdint.h>

constexpr uint8_t FC_PID_TERM_COUNT = 12;
constexpr uint8_t FC_MIXER_COLUMNS = 4;
constexpr uint8_t FC_MIXER_MAX_MOTORS = 8;
constexpr uint8_t FC_MIXER_VALUE_COUNT = FC_MIXER_COLUMNS * FC_MIXER_MAX_MOTORS;
constexpr uint8_t FC_ORIENTATION_VALUE_COUNT = 9;
constexpr uint8_t FC_PWM_OUTPUT_MAX_COUNT = 16;
constexpr uint8_t FC_MODE_CHANNEL_MIN = 4; // CH5, zero-based
constexpr uint8_t FC_MODE_CHANNEL_MAX = 15; // CH16, zero-based
constexpr uint8_t FC_MODE_CHANNEL_DEFAULT = 5; // CH6, zero-based
constexpr uint8_t FC_ARM_CHANNEL_DEFAULT = 4; // CH5, zero-based
constexpr uint16_t FC_CHANNEL_RANGE_MIN_US = 900;
constexpr uint16_t FC_CHANNEL_RANGE_MAX_US = 2100;
constexpr uint16_t FC_PWM_LIMIT_MIN_US = 500;
constexpr uint16_t FC_PWM_LIMIT_MAX_US = 2500;
constexpr uint16_t FC_PWM_LIMIT_DEFAULT_MIN_US = 1000;
constexpr uint16_t FC_PWM_LIMIT_DEFAULT_CENTER_US = 1500;
constexpr uint16_t FC_PWM_LIMIT_DEFAULT_MAX_US = 2000;

enum FlightControlMode : uint8_t
{
    FLIGHT_CONTROL_MODE_MANUAL = 0,
    FLIGHT_CONTROL_MODE_RATE,
    FLIGHT_CONTROL_MODE_ANGLE,
    FLIGHT_CONTROL_MODE_COUNT,
};

struct FlightControlChannelRange
{
    uint16_t startUs;
    uint16_t endUs;
};

struct FlightControlPwmOutputLimits
{
    uint16_t minUs;
    uint16_t centerUs;
    uint16_t maxUs;
};

inline bool FlightControlRangeIsActive(const FlightControlChannelRange &range, uint16_t channelUs)
{
    return range.startUs < range.endUs && channelUs >= range.startUs && channelUs < range.endUs;
}

class FlightControlConfig
{
public:
    void Load();
    bool Commit();
    void SetDefaults();

    const int16_t *GetRatePid() const { return m_ratePid; }
    const int16_t *GetAnglePid() const { return m_anglePid; }
    bool GetModeEnabled(FlightControlMode mode) const { return mode != FLIGHT_CONTROL_MODE_MANUAL && m_modeEnabled[mode]; }
    uint8_t GetModeChannel(FlightControlMode mode) const { return m_modeChannels[mode]; }
    const FlightControlChannelRange &GetModeRange(FlightControlMode mode) const { return m_modeRanges[mode]; }
    bool GetArmMode() const { return m_armMode; }
    uint8_t GetArmChannel() const { return m_armChannel; }
    const FlightControlChannelRange &GetArmRange() const { return m_armRange; }
    const float *GetMixer() const { return m_mixer; }
    uint8_t GetMixerCount() const { return m_mixerCount; }
    bool GetMixerOutputServo(uint8_t output) const
    {
        return output < FC_MIXER_MAX_MOTORS && m_mixerOutputServo[output];
    }
    const float *GetOrientation() const { return m_orientation; }
    const FlightControlPwmOutputLimits &GetPwmOutputLimits(uint8_t output) const { return m_pwmOutputLimits[output]; }
    bool GetPwmOutputWifiEnabled() const { return m_pwmOutputWifiEnabled; }
    uint16_t GetPwmOutputWifiValue(uint8_t output) const { return m_pwmOutputWifiValues[output]; }
    bool IsModified() const { return m_modified; }

    void SetRatePid(uint8_t index, int16_t value);
    void SetAnglePid(uint8_t index, int16_t value);
    void SetModeEnabled(FlightControlMode mode, bool enabled);
    void SetModeChannel(FlightControlMode mode, uint8_t channel);
    void SetModeRange(FlightControlMode mode, uint16_t startUs, uint16_t endUs);
    void SetArmMode(bool enabled);
    void SetArmChannel(uint8_t channel);
    void SetArmRange(uint16_t startUs, uint16_t endUs);
    void SetMixer(const float *values, uint8_t count);
    void SetMixerOutputServo(uint8_t output, bool isServo);
    void SetOrientation(const float *values, uint8_t count);
    void SetPwmOutputLimits(uint8_t output, uint16_t minUs, uint16_t centerUs, uint16_t maxUs);
    void SetPwmOutputWifiEnabled(bool enabled);
    void SetPwmOutputWifiValue(uint8_t output, uint16_t valueUs);

private:
    int16_t m_ratePid[FC_PID_TERM_COUNT] = {};
    int16_t m_anglePid[FC_PID_TERM_COUNT] = {};
    float m_mixer[FC_MIXER_VALUE_COUNT] = {};
    bool m_mixerOutputServo[FC_MIXER_MAX_MOTORS] = {};
    float m_orientation[FC_ORIENTATION_VALUE_COUNT] = {};
    FlightControlPwmOutputLimits m_pwmOutputLimits[FC_PWM_OUTPUT_MAX_COUNT] = {};
    bool m_pwmOutputWifiEnabled = false;
    uint16_t m_pwmOutputWifiValues[FC_PWM_OUTPUT_MAX_COUNT] = {};
    uint8_t m_mixerCount = 0;
    bool m_modeEnabled[FLIGHT_CONTROL_MODE_COUNT] = {};
    uint8_t m_modeChannels[FLIGHT_CONTROL_MODE_COUNT] = {};
    FlightControlChannelRange m_modeRanges[FLIGHT_CONTROL_MODE_COUNT] = {};
    bool m_armMode = false;
    uint8_t m_armChannel = FC_ARM_CHANNEL_DEFAULT;
    FlightControlChannelRange m_armRange = {};
    bool m_modified = false;
};

extern FlightControlConfig flightControlConfig;

#endif
