#include "FlightControlConfig.h"

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

#include <ArduinoJson.h>
#include <FS.h>
#if defined(PLATFORM_ESP32)
#include <SPIFFS.h>
#endif
#include <math.h>
#include <string.h>

static constexpr char FC_CONFIG_FILE[] = "/fc.json";
static constexpr char FC_CONFIG_TEMP_FILE[] = "/fc.json.tmp";

FlightControlConfig flightControlConfig;

void FlightControlConfig::SetDefaults()
{
    memset(m_ratePid, 0, sizeof(m_ratePid));
    memset(m_anglePid, 0, sizeof(m_anglePid));
    for (uint8_t axis = 0; axis < FC_ANGLE_RATE_LIMIT_AXIS_COUNT; ++axis)
    {
        m_angleRateLimitDps[axis] = FC_ANGLE_RATE_LIMIT_DEFAULT_DPS;
    }
    m_dtermLpfHz = FC_DTERM_LPF_DEFAULT_HZ;
    m_gyroLpfHz = FC_GYRO_LPF_DEFAULT_HZ;
    m_gyroBiasMode = FLIGHT_CONTROL_GYRO_BIAS_CONFIGURED;
    memset(m_mixer, 0, sizeof(m_mixer));
    memset(m_mixerOutputServo, 0, sizeof(m_mixerOutputServo));
    memset(m_orientation, 0, sizeof(m_orientation));
    memset(m_gyroBias, 0, sizeof(m_gyroBias));
    memset(m_accelBias, 0, sizeof(m_accelBias));
    for (uint8_t axis = 0; axis < FC_IMU_AXIS_COUNT; ++axis) m_accelScale[axis] = 1.0f;
    for (uint8_t output = 0; output < FC_PWM_OUTPUT_MAX_COUNT; ++output)
    {
        m_pwmOutputLimits[output] = {
            FC_PWM_LIMIT_DEFAULT_MIN_US,
            FC_PWM_LIMIT_DEFAULT_CENTER_US,
            FC_PWM_LIMIT_DEFAULT_MAX_US,
        };
        m_pwmOutputWifiValues[output] = FC_PWM_LIMIT_DEFAULT_CENTER_US;
    }
    for (uint8_t i = 0; i < FC_ORIENTATION_VALUE_COUNT; ++i)
    {
        m_orientation[i] = (i % 4) == 0 ? 1.0f : 0.0f;
    }
    m_mixerCount = 0;
    m_pwmOutputWifiEnabled = false;
    for (uint8_t mode = 0; mode < FLIGHT_CONTROL_MODE_COUNT; ++mode)
    {
        m_modeEnabled[mode] = mode == FLIGHT_CONTROL_MODE_RATE;
        m_modeChannels[mode] = FC_MODE_CHANNEL_DEFAULT;
    }
    m_modeRanges[FLIGHT_CONTROL_MODE_RATE] = {1300, 1700};
    m_modeRanges[FLIGHT_CONTROL_MODE_ANGLE] = {1700, 2100};
    m_wifiCoexistEnabled = false;
    m_wifiCoexistChannel = FC_WIFI_CHANNEL_DEFAULT;
    m_wifiCoexistRange = {1700, 2100};
    m_armMode = false;
    m_armChannel = FC_ARM_CHANNEL_DEFAULT;
    m_armRange = {1700, 2100};
    m_modified = true;
}

void FlightControlConfig::Load()
{
    SetDefaults();

    File file = SPIFFS.open(FC_CONFIG_FILE, "r");
    if (!file)
    {
        Commit();
        return;
    }

    JsonDocument doc;
    const DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (error)
    {
        Commit();
        return;
    }

    JsonArray ratePid = doc["rate_pid"].as<JsonArray>();
    for (uint8_t i = 0; i < min(ratePid.size(), (size_t)FC_PID_TERM_COUNT); ++i)
    {
        m_ratePid[i] = ratePid[i].as<int16_t>();
    }

    JsonArray anglePid = doc["angle_pid"].as<JsonArray>();
    for (uint8_t i = 0; i < min(anglePid.size(), (size_t)FC_PID_TERM_COUNT); ++i)
    {
        m_anglePid[i] = anglePid[i].as<int16_t>();
    }
    JsonArray angleRateLimits = doc["angle_rate_limits_dps"].as<JsonArray>();
    for (uint8_t axis = 0; axis < min(angleRateLimits.size(), (size_t)FC_ANGLE_RATE_LIMIT_AXIS_COUNT); ++axis)
    {
        SetAngleRateLimitDps(axis, angleRateLimits[axis].as<int>());
    }
    SetDtermLpfHz(doc["dterm_lpf_hz"] | FC_DTERM_LPF_DEFAULT_HZ);
    SetGyroLpfHz(doc["gyro_lpf_hz"] | FC_GYRO_LPF_DEFAULT_HZ);
    SetGyroBiasMode((FlightControlGyroBiasMode)(doc["gyro_bias_mode"] | FLIGHT_CONTROL_GYRO_BIAS_CONFIGURED));

    JsonVariant modeConditionsValue = doc["mode_conditions"];
    if (modeConditionsValue.is<JsonObject>())
    {
        JsonObject modeConditions = modeConditionsValue.as<JsonObject>();
        const char *keys[] = {nullptr, "rate", "angle"};
        for (uint8_t mode = FLIGHT_CONTROL_MODE_RATE; mode < FLIGHT_CONTROL_MODE_COUNT; ++mode)
        {
            JsonArray condition = modeConditions[keys[mode]].as<JsonArray>();
            SetModeEnabled((FlightControlMode)mode, condition.size() >= 3);
            if (condition.size() >= 3)
            {
                const int channel = condition[0].as<int>();
                SetModeChannel((FlightControlMode)mode,
                    (uint8_t)constrain(channel - 1, FC_MODE_CHANNEL_MIN, FC_MODE_CHANNEL_MAX));
                SetModeRange((FlightControlMode)mode, condition[1].as<uint16_t>(), condition[2].as<uint16_t>());
            }
        }
    }
    else if (modeConditionsValue.is<JsonArray>())
    {
        // Migrate the earlier three-entry [MANUAL, RATE, ANGLE] format.
        JsonArray modeConditions = modeConditionsValue.as<JsonArray>();
        for (uint8_t mode = FLIGHT_CONTROL_MODE_RATE; mode < FLIGHT_CONTROL_MODE_COUNT && mode < modeConditions.size(); ++mode)
        {
            JsonArray condition = modeConditions[mode].as<JsonArray>();
            SetModeEnabled((FlightControlMode)mode, condition.size() >= 3);
            if (condition.size() >= 3)
            {
                const int channel = condition[0].as<int>();
                SetModeChannel((FlightControlMode)mode,
                    (uint8_t)constrain(channel - 1, FC_MODE_CHANNEL_MIN, FC_MODE_CHANNEL_MAX));
                SetModeRange((FlightControlMode)mode, condition[1].as<uint16_t>(), condition[2].as<uint16_t>());
            }
        }
    }
    else
    {
        // Migrate the earlier shared-channel format. Configurations from
        // firmware without mode support default to RATE only; preserve the
        // old angle_enabled setting only when it was explicitly enabled.
        const bool hasSharedModeConfig = doc.containsKey("mode_channel") || doc.containsKey("mode_ranges");
        const int modeChannel = doc["mode_channel"] | (FC_MODE_CHANNEL_DEFAULT + 1);
        JsonArray modeRanges = doc["mode_ranges"].as<JsonArray>();
        for (uint8_t mode = FLIGHT_CONTROL_MODE_RATE; mode < FLIGHT_CONTROL_MODE_COUNT; ++mode)
        {
            const bool enabled = mode == FLIGHT_CONTROL_MODE_RATE ||
                (mode == FLIGHT_CONTROL_MODE_ANGLE && (hasSharedModeConfig || (doc["angle_enabled"] | false)));
            SetModeEnabled((FlightControlMode)mode, enabled);
            SetModeChannel((FlightControlMode)mode, (uint8_t)constrain(modeChannel - 1, FC_MODE_CHANNEL_MIN, FC_MODE_CHANNEL_MAX));
            if (mode < modeRanges.size())
            {
                JsonArray range = modeRanges[mode].as<JsonArray>();
                if (range.size() >= 2)
                {
                    SetModeRange((FlightControlMode)mode, range[0].as<uint16_t>(), range[1].as<uint16_t>());
                }
            }
        }
    }
    JsonObject wifiConditions = doc["wifi_conditions"].as<JsonObject>();
    if (!wifiConditions.isNull())
    {
        JsonArray condition = wifiConditions["coexist"].as<JsonArray>();
        SetWifiCoexistEnabled(condition.size() >= 3);
        if (condition.size() >= 3)
        {
            const int channel = condition[0].as<int>();
            SetWifiCoexistChannel((uint8_t)constrain(channel - 1, FC_MODE_CHANNEL_MIN, FC_MODE_CHANNEL_MAX));
            SetWifiCoexistRange(condition[1].as<uint16_t>(), condition[2].as<uint16_t>());
        }
    }
    m_armMode = doc["arm_enabled"] | false;
    const int armChannel = doc["arm_channel"] | (FC_ARM_CHANNEL_DEFAULT + 1);
    m_armChannel = (uint8_t)constrain(armChannel - 1, FC_MODE_CHANNEL_MIN, FC_MODE_CHANNEL_MAX);
    JsonArray armRange = doc["arm_range"].as<JsonArray>();
    if (armRange.size() >= 2)
    {
        SetArmRange(armRange[0].as<uint16_t>(), armRange[1].as<uint16_t>());
    }

    JsonArray mixer = doc["mixer"].as<JsonArray>();
    m_mixerCount = min(mixer.size(), (size_t)FC_MIXER_VALUE_COUNT);
    m_mixerCount -= m_mixerCount % FC_MIXER_COLUMNS;
    for (uint8_t i = 0; i < m_mixerCount; ++i)
    {
        m_mixer[i] = mixer[i].as<float>();
    }
    JsonArray mixerServos = doc["mixer_servos"].as<JsonArray>();
    const uint8_t mixerOutputCount = m_mixerCount / FC_MIXER_COLUMNS;
    for (uint8_t output = 0; output < min(mixerServos.size(), (size_t)mixerOutputCount); ++output)
    {
        m_mixerOutputServo[output] = mixerServos[output].as<bool>();
    }

    JsonArray orientation = doc["orientation"].as<JsonArray>();
    if (orientation.size() >= FC_ORIENTATION_VALUE_COUNT)
    {
        for (uint8_t i = 0; i < FC_ORIENTATION_VALUE_COUNT; ++i)
        {
            m_orientation[i] = orientation[i].as<float>();
        }
    }
    JsonArray gyroBias = doc["gyro_bias"].as<JsonArray>();
    if (gyroBias.size() >= FC_IMU_AXIS_COUNT)
    {
        float values[FC_IMU_AXIS_COUNT];
        for (uint8_t axis = 0; axis < FC_IMU_AXIS_COUNT; ++axis) values[axis] = gyroBias[axis].as<float>();
        SetGyroBias(values, FC_IMU_AXIS_COUNT);
    }
    JsonArray accelBias = doc["accel_bias"].as<JsonArray>();
    if (accelBias.size() >= FC_IMU_AXIS_COUNT)
    {
        float values[FC_IMU_AXIS_COUNT];
        for (uint8_t axis = 0; axis < FC_IMU_AXIS_COUNT; ++axis) values[axis] = accelBias[axis].as<float>();
        SetAccelBias(values, FC_IMU_AXIS_COUNT);
    }
    JsonArray accelScale = doc["accel_scale"].as<JsonArray>();
    for (uint8_t axis = 0; axis < min(accelScale.size(), (size_t)FC_IMU_AXIS_COUNT); ++axis)
    {
        const float value = accelScale[axis].as<float>();
        if (isfinite(value) && value > 0.5f && value < 1.5f) m_accelScale[axis] = value;
    }

    JsonArray pwmOutputLimits = doc["pwm_output_limits"].as<JsonArray>();
    for (uint8_t output = 0; output < min(pwmOutputLimits.size(), (size_t)FC_PWM_OUTPUT_MAX_COUNT); ++output)
    {
        JsonArray limits = pwmOutputLimits[output].as<JsonArray>();
        if (limits.size() >= 3)
        {
            SetPwmOutputLimits(output,
                limits[0].as<uint16_t>(),
                limits[1].as<uint16_t>(),
                limits[2].as<uint16_t>());
        }
    }
    for (uint8_t output = 0; output < FC_PWM_OUTPUT_MAX_COUNT; ++output)
    {
        m_pwmOutputWifiValues[output] = m_pwmOutputLimits[output].centerUs;
    }
    m_modified = false;
}

bool FlightControlConfig::Commit()
{
    if (!m_modified)
    {
        return true;
    }

    JsonDocument doc;
    copyArray(m_ratePid, doc["rate_pid"].to<JsonArray>());
    copyArray(m_anglePid, doc["angle_pid"].to<JsonArray>());
    copyArray(m_angleRateLimitDps, doc["angle_rate_limits_dps"].to<JsonArray>());
    doc["dterm_lpf_hz"] = m_dtermLpfHz;
    doc["gyro_lpf_hz"] = m_gyroLpfHz;
    doc["gyro_bias_mode"] = m_gyroBiasMode;
    JsonObject modeConditions = doc["mode_conditions"].to<JsonObject>();
    const char *keys[] = {nullptr, "rate", "angle"};
    for (uint8_t mode = FLIGHT_CONTROL_MODE_RATE; mode < FLIGHT_CONTROL_MODE_COUNT; ++mode)
    {
        if (!m_modeEnabled[mode])
        {
            continue;
        }
        JsonArray condition = modeConditions[keys[mode]].to<JsonArray>();
        condition.add(m_modeChannels[mode] + 1);
        condition.add(m_modeRanges[mode].startUs);
        condition.add(m_modeRanges[mode].endUs);
    }
    JsonObject wifiConditions = doc["wifi_conditions"].to<JsonObject>();
    if (m_wifiCoexistEnabled)
    {
        JsonArray condition = wifiConditions["coexist"].to<JsonArray>();
        condition.add(m_wifiCoexistChannel + 1);
        condition.add(m_wifiCoexistRange.startUs);
        condition.add(m_wifiCoexistRange.endUs);
    }
    doc["arm_enabled"] = m_armMode;
    doc["arm_channel"] = m_armChannel + 1;
    JsonArray armRange = doc["arm_range"].to<JsonArray>();
    armRange.add(m_armRange.startUs);
    armRange.add(m_armRange.endUs);
    copyArray(m_mixer, m_mixerCount, doc["mixer"].to<JsonArray>());
    JsonArray mixerServos = doc["mixer_servos"].to<JsonArray>();
    for (uint8_t output = 0; output < m_mixerCount / FC_MIXER_COLUMNS; ++output)
    {
        mixerServos.add(m_mixerOutputServo[output]);
    }
    copyArray(m_orientation, doc["orientation"].to<JsonArray>());
    copyArray(m_gyroBias, doc["gyro_bias"].to<JsonArray>());
    copyArray(m_accelBias, doc["accel_bias"].to<JsonArray>());
    copyArray(m_accelScale, doc["accel_scale"].to<JsonArray>());
    JsonArray pwmOutputLimits = doc["pwm_output_limits"].to<JsonArray>();
    for (uint8_t output = 0; output < FC_PWM_OUTPUT_MAX_COUNT; ++output)
    {
        JsonArray limits = pwmOutputLimits.add<JsonArray>();
        limits.add(m_pwmOutputLimits[output].minUs);
        limits.add(m_pwmOutputLimits[output].centerUs);
        limits.add(m_pwmOutputLimits[output].maxUs);
    }
    File file = SPIFFS.open(FC_CONFIG_TEMP_FILE, "w");
    if (!file)
    {
        return false;
    }
    const size_t written = serializeJson(doc, file);
    file.close();
    if (written == 0)
    {
        SPIFFS.remove(FC_CONFIG_TEMP_FILE);
        return false;
    }

    SPIFFS.remove(FC_CONFIG_FILE);
    if (!SPIFFS.rename(FC_CONFIG_TEMP_FILE, FC_CONFIG_FILE))
    {
        return false;
    }
    m_modified = false;
    return true;
}

void FlightControlConfig::SetRatePid(uint8_t index, int16_t value)
{
    if (index < FC_PID_TERM_COUNT && m_ratePid[index] != value)
    {
        m_ratePid[index] = value;
        m_modified = true;
    }
}

void FlightControlConfig::SetAnglePid(uint8_t index, int16_t value)
{
    if (index < FC_PID_TERM_COUNT && m_anglePid[index] != value)
    {
        m_anglePid[index] = value;
        m_modified = true;
    }
}

void FlightControlConfig::SetDtermLpfHz(int frequencyHz)
{
    const uint8_t clippedFrequency = frequencyHz <= 0
        ? 0
        : (uint8_t)constrain(frequencyHz, (int)FC_DTERM_LPF_MIN_HZ, (int)FC_DTERM_LPF_MAX_HZ);
    if (m_dtermLpfHz != clippedFrequency)
    {
        m_dtermLpfHz = clippedFrequency;
        m_modified = true;
    }
}

void FlightControlConfig::SetGyroLpfHz(int frequencyHz)
{
    const uint8_t clippedFrequency = frequencyHz <= 0
        ? 0
        : (uint8_t)constrain(frequencyHz, (int)FC_GYRO_LPF_MIN_HZ, (int)FC_GYRO_LPF_MAX_HZ);
    if (m_gyroLpfHz != clippedFrequency)
    {
        m_gyroLpfHz = clippedFrequency;
        m_modified = true;
    }
}

void FlightControlConfig::SetModeEnabled(FlightControlMode mode, bool enabled)
{
    if (mode == FLIGHT_CONTROL_MODE_MANUAL || mode >= FLIGHT_CONTROL_MODE_COUNT)
    {
        return;
    }
    if (m_modeEnabled[mode] != enabled)
    {
        m_modeEnabled[mode] = enabled;
        m_modified = true;
    }
}

void FlightControlConfig::SetModeChannel(FlightControlMode mode, uint8_t channel)
{
    if (mode >= FLIGHT_CONTROL_MODE_COUNT)
    {
        return;
    }
    const uint8_t clippedChannel = constrain(channel, FC_MODE_CHANNEL_MIN, FC_MODE_CHANNEL_MAX);
    if (m_modeChannels[mode] != clippedChannel)
    {
        m_modeChannels[mode] = clippedChannel;
        m_modified = true;
    }
}

void FlightControlConfig::SetModeRange(FlightControlMode mode, uint16_t startUs, uint16_t endUs)
{
    if (mode >= FLIGHT_CONTROL_MODE_COUNT)
    {
        return;
    }
    FlightControlChannelRange range = {
        (uint16_t)constrain(startUs, FC_CHANNEL_RANGE_MIN_US, FC_CHANNEL_RANGE_MAX_US),
        (uint16_t)constrain(endUs, FC_CHANNEL_RANGE_MIN_US, FC_CHANNEL_RANGE_MAX_US),
    };
    if (memcmp(&m_modeRanges[mode], &range, sizeof(range)) != 0)
    {
        m_modeRanges[mode] = range;
        m_modified = true;
    }
}

void FlightControlConfig::SetArmMode(bool enabled)
{
    if (m_armMode != enabled)
    {
        m_armMode = enabled;
        m_modified = true;
    }
}

void FlightControlConfig::SetGyroBiasMode(FlightControlGyroBiasMode mode)
{
    if (mode >= FLIGHT_CONTROL_GYRO_BIAS_MODE_COUNT)
    {
        mode = FLIGHT_CONTROL_GYRO_BIAS_CONFIGURED;
    }
    if (m_gyroBiasMode != mode)
    {
        m_gyroBiasMode = mode;
        m_modified = true;
    }
}

void FlightControlConfig::SetWifiCoexistEnabled(bool enabled)
{
    if (m_wifiCoexistEnabled != enabled)
    {
        m_wifiCoexistEnabled = enabled;
        m_modified = true;
    }
}

void FlightControlConfig::SetAngleRateLimitDps(uint8_t axis, int valueDps)
{
    if (axis >= FC_ANGLE_RATE_LIMIT_AXIS_COUNT)
    {
        return;
    }
    const uint16_t clippedValue = (uint16_t)constrain(
        valueDps,
        (int)FC_ANGLE_RATE_LIMIT_MIN_DPS,
        (int)FC_ANGLE_RATE_LIMIT_MAX_DPS);
    if (m_angleRateLimitDps[axis] != clippedValue)
    {
        m_angleRateLimitDps[axis] = clippedValue;
        m_modified = true;
    }
}

void FlightControlConfig::SetWifiCoexistChannel(uint8_t channel)
{
    const uint8_t clippedChannel = constrain(channel, FC_MODE_CHANNEL_MIN, FC_MODE_CHANNEL_MAX);
    if (m_wifiCoexistChannel != clippedChannel)
    {
        m_wifiCoexistChannel = clippedChannel;
        m_modified = true;
    }
}

void FlightControlConfig::SetWifiCoexistRange(uint16_t startUs, uint16_t endUs)
{
    FlightControlChannelRange range = {
        (uint16_t)constrain(startUs, FC_CHANNEL_RANGE_MIN_US, FC_CHANNEL_RANGE_MAX_US),
        (uint16_t)constrain(endUs, FC_CHANNEL_RANGE_MIN_US, FC_CHANNEL_RANGE_MAX_US),
    };
    if (memcmp(&m_wifiCoexistRange, &range, sizeof(range)) != 0)
    {
        m_wifiCoexistRange = range;
        m_modified = true;
    }
}

void FlightControlConfig::SetArmChannel(uint8_t channel)
{
    const uint8_t clippedChannel = constrain(channel, FC_MODE_CHANNEL_MIN, FC_MODE_CHANNEL_MAX);
    if (m_armChannel != clippedChannel)
    {
        m_armChannel = clippedChannel;
        m_modified = true;
    }
}

void FlightControlConfig::SetArmRange(uint16_t startUs, uint16_t endUs)
{
    FlightControlChannelRange range = {
        (uint16_t)constrain(startUs, FC_CHANNEL_RANGE_MIN_US, FC_CHANNEL_RANGE_MAX_US),
        (uint16_t)constrain(endUs, FC_CHANNEL_RANGE_MIN_US, FC_CHANNEL_RANGE_MAX_US),
    };
    if (memcmp(&m_armRange, &range, sizeof(range)) != 0)
    {
        m_armRange = range;
        m_modified = true;
    }
}

void FlightControlConfig::SetMixer(const float *values, uint8_t count)
{
    uint8_t clippedCount = min(count, FC_MIXER_VALUE_COUNT);
    clippedCount -= clippedCount % FC_MIXER_COLUMNS;
    if (m_mixerCount != clippedCount || memcmp(m_mixer, values, clippedCount * sizeof(float)) != 0)
    {
        memcpy(m_mixer, values, clippedCount * sizeof(float));
        memset(m_mixer + clippedCount, 0, (FC_MIXER_VALUE_COUNT - clippedCount) * sizeof(float));
        m_mixerCount = clippedCount;
        m_modified = true;
    }
}

void FlightControlConfig::SetMixerOutputServo(uint8_t output, bool isServo)
{
    if (output < FC_MIXER_MAX_MOTORS && m_mixerOutputServo[output] != isServo)
    {
        m_mixerOutputServo[output] = isServo;
        m_modified = true;
    }
}

void FlightControlConfig::SetOrientation(const float *values, uint8_t count)
{
    if (count >= FC_ORIENTATION_VALUE_COUNT && memcmp(m_orientation, values, sizeof(m_orientation)) != 0)
    {
        memcpy(m_orientation, values, sizeof(m_orientation));
        m_modified = true;
    }
}

static bool setImuVector(float *target, const float *values, uint8_t count, bool positiveOnly)
{
    if (!values || count < FC_IMU_AXIS_COUNT) return false;
    float next[FC_IMU_AXIS_COUNT];
    for (uint8_t axis = 0; axis < FC_IMU_AXIS_COUNT; ++axis)
    {
        next[axis] = values[axis];
        if (!isfinite(next[axis]) || (positiveOnly && (next[axis] <= 0.5f || next[axis] >= 1.5f))) return false;
    }
    if (memcmp(target, next, sizeof(next)) == 0) return false;
    memcpy(target, next, sizeof(next));
    return true;
}

void FlightControlConfig::SetGyroBias(const float *values, uint8_t count)
{
    if (!values || count < FC_IMU_AXIS_COUNT) return;
    for (uint8_t axis = 0; axis < FC_IMU_AXIS_COUNT; ++axis)
    {
        if (!isfinite(values[axis]) || fabsf(values[axis]) > 100.0f) return;
    }
    if (setImuVector(m_gyroBias, values, count, false)) m_modified = true;
}

void FlightControlConfig::SetAccelBias(const float *values, uint8_t count)
{
    if (!values || count < FC_IMU_AXIS_COUNT) return;
    for (uint8_t axis = 0; axis < FC_IMU_AXIS_COUNT; ++axis)
    {
        if (!isfinite(values[axis]) || fabsf(values[axis]) > 20.0f) return;
    }
    if (setImuVector(m_accelBias, values, count, false)) m_modified = true;
}

void FlightControlConfig::SetAccelScale(const float *values, uint8_t count)
{
    if (setImuVector(m_accelScale, values, count, true)) m_modified = true;
}

void FlightControlConfig::SetPwmOutputLimits(uint8_t output, uint16_t minUs, uint16_t centerUs, uint16_t maxUs)
{
    if (output >= FC_PWM_OUTPUT_MAX_COUNT ||
        minUs < FC_PWM_LIMIT_MIN_US || maxUs > FC_PWM_LIMIT_MAX_US ||
        minUs >= centerUs || centerUs >= maxUs)
    {
        return;
    }

    const FlightControlPwmOutputLimits limits = {minUs, centerUs, maxUs};
    if (memcmp(&m_pwmOutputLimits[output], &limits, sizeof(limits)) != 0)
    {
        m_pwmOutputLimits[output] = limits;
        m_pwmOutputWifiValues[output] = constrain(m_pwmOutputWifiValues[output], minUs, maxUs);
        m_modified = true;
    }
}

void FlightControlConfig::SetPwmOutputWifiEnabled(bool enabled)
{
    m_pwmOutputWifiEnabled = enabled;
}

void FlightControlConfig::SetPwmOutputWifiValue(uint8_t output, uint16_t valueUs)
{
    if (output >= FC_PWM_OUTPUT_MAX_COUNT)
    {
        return;
    }
    const FlightControlPwmOutputLimits &limits = m_pwmOutputLimits[output];
    m_pwmOutputWifiValues[output] = constrain(valueUs, limits.minUs, limits.maxUs);
}

#endif
