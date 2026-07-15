#include "FlightControlConfig.h"

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)

#include <ArduinoJson.h>
#include <FS.h>
#if defined(PLATFORM_ESP32)
#include <SPIFFS.h>
#endif
#include <string.h>

static constexpr char FC_CONFIG_FILE[] = "/fc.json";
static constexpr char FC_CONFIG_TEMP_FILE[] = "/fc.json.tmp";

FlightControlConfig flightControlConfig;

void FlightControlConfig::SetDefaults()
{
    memset(m_ratePid, 0, sizeof(m_ratePid));
    memset(m_anglePid, 0, sizeof(m_anglePid));
    memset(m_mixer, 0, sizeof(m_mixer));
    memset(m_orientation, 0, sizeof(m_orientation));
    for (uint8_t i = 0; i < FC_ORIENTATION_VALUE_COUNT; ++i)
    {
        m_orientation[i] = (i % 4) == 0 ? 1.0f : 0.0f;
    }
    m_mixerCount = 0;
    for (uint8_t mode = 0; mode < FLIGHT_CONTROL_MODE_COUNT; ++mode)
    {
        m_modeEnabled[mode] = mode == FLIGHT_CONTROL_MODE_RATE;
        m_modeChannels[mode] = FC_MODE_CHANNEL_DEFAULT;
    }
    m_modeRanges[FLIGHT_CONTROL_MODE_RATE] = {1300, 1700};
    m_modeRanges[FLIGHT_CONTROL_MODE_ANGLE] = {1700, 2100};
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

    JsonArray orientation = doc["orientation"].as<JsonArray>();
    if (orientation.size() >= FC_ORIENTATION_VALUE_COUNT)
    {
        for (uint8_t i = 0; i < FC_ORIENTATION_VALUE_COUNT; ++i)
        {
            m_orientation[i] = orientation[i].as<float>();
        }
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
    doc["arm_enabled"] = m_armMode;
    doc["arm_channel"] = m_armChannel + 1;
    JsonArray armRange = doc["arm_range"].to<JsonArray>();
    armRange.add(m_armRange.startUs);
    armRange.add(m_armRange.endUs);
    copyArray(m_mixer, m_mixerCount, doc["mixer"].to<JsonArray>());
    copyArray(m_orientation, doc["orientation"].to<JsonArray>());

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

void FlightControlConfig::SetOrientation(const float *values, uint8_t count)
{
    if (count >= FC_ORIENTATION_VALUE_COUNT && memcmp(m_orientation, values, sizeof(m_orientation)) != 0)
    {
        memcpy(m_orientation, values, sizeof(m_orientation));
        m_modified = true;
    }
}

#endif
