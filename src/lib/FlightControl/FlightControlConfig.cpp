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
    m_angleMode = false;
    m_armMode = false;
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

    m_angleMode = doc["angle_enabled"] | false;
    m_armMode = doc["arm_enabled"] | false;

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
    doc["angle_enabled"] = m_angleMode;
    doc["arm_enabled"] = m_armMode;
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

void FlightControlConfig::SetAngleMode(bool enabled)
{
    if (m_angleMode != enabled)
    {
        m_angleMode = enabled;
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
