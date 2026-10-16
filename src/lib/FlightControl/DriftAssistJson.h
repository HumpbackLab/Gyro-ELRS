#pragma once
#include "DriftAssist.h"
#include <ArduinoJson.h>

inline void driftSettingsToJson(JsonObject obj, const DriftAssistSettings &s)
{
    obj["enabled"] = s.enabled;
    obj["channel"] = s.channel;
    JsonArray range = obj["range"].to<JsonArray>();
    range.add(s.startUs); range.add(s.endUs);
    obj["threshold_dps"] = s.thresholdDps;
    obj["ramp_dps"] = s.rampDps;
    obj["max_reduction_percent"] = s.maxReductionPercent;
    obj["attack_ms"] = s.attackMs;
    obj["recovery_ms"] = s.recoveryMs;
}

inline bool driftSettingsFromJson(JsonVariantConst obj, DriftAssistSettings &s)
{
    if (!obj.is<JsonObjectConst>() || !obj["enabled"].is<bool>() ||
        !obj["channel"].is<int>() || !obj["range"].is<JsonArrayConst>() ||
        obj["range"].size() != 2 || !obj["range"][0].is<int>() || !obj["range"][1].is<int>() ||
        !obj["threshold_dps"].is<float>() || !obj["ramp_dps"].is<float>() ||
        !obj["max_reduction_percent"].is<float>() || !obj["attack_ms"].is<int>() || !obj["recovery_ms"].is<int>()) return false;
    const int channel = obj["channel"], start = obj["range"][0], end = obj["range"][1];
    const int attack = obj["attack_ms"], recovery = obj["recovery_ms"];
    if (channel < 5 || channel > 16 || start < 900 || end > 2100 || start >= end ||
        attack < 50 || attack > 500 || recovery < 100 || recovery > 1000) return false;
    DriftAssistSettings next;
    next.enabled = obj["enabled"]; next.channel = channel;
    next.startUs = start; next.endUs = end;
    next.thresholdDps = obj["threshold_dps"]; next.rampDps = obj["ramp_dps"];
    next.maxReductionPercent = obj["max_reduction_percent"];
    next.attackMs = attack; next.recoveryMs = recovery;
    if (!driftSettingsValid(next)) return false;
    s = next;
    return true;
}
