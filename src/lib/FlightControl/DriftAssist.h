#pragma once

#include <cmath>
#include <stdint.h>

struct DriftAssistSettings {
    bool enabled = false;
    uint8_t channel = 8; // One-based RC channel, like the HTTP configuration.
    uint16_t startUs = 1700;
    uint16_t endUs = 2100;
    float thresholdDps = 20.0f;
    float rampDps = 80.0f;
    float maxReductionPercent = 15.0f;
    uint16_t attackMs = 120;
    uint16_t recoveryMs = 300;
};

inline bool driftSettingsValid(const DriftAssistSettings &s)
{
    return s.channel >= 5 && s.channel <= 16 && s.startUs >= 900 &&
        s.endUs <= 2100 && s.startUs < s.endUs &&
        std::isfinite(s.thresholdDps) && s.thresholdDps >= 5 && s.thresholdDps <= 100 &&
        std::isfinite(s.rampDps) && s.rampDps >= 20 && s.rampDps <= 300 &&
        std::isfinite(s.maxReductionPercent) && s.maxReductionPercent >= 0 && s.maxReductionPercent <= 30 &&
        s.attackMs >= 50 && s.attackMs <= 500 && s.recoveryMs >= 100 && s.recoveryMs <= 1000;
}

// Allow zero-throttle changes while armed so Wi-Fi/RC coexistence can stay on.
// Use loadStickTargets()' endpoint and reject unavailable/stale RC state.
inline const char *driftSettingsChangeError(bool changed, bool connected,
    bool stateValid, bool armed, uint16_t throttleUs)
{
    if (!changed || !connected) return nullptr;
    if (!stateValid) return "Wait for valid RC state before changing drift assistance";
    if (!armed) return nullptr;
    if (throttleUs > 988) return "Lower throttle to zero before changing drift assistance";
    return nullptr;
}

inline bool driftMixerCompatible(const float *mix, const bool *servos, unsigned count)
{
    if (!mix || !servos || !count || count > 32 || count % 4) return false;
    bool throttle = false, steering = false;
    for (unsigned i = 0; i < count / 4; ++i) {
        const float *row = mix + i * 4;
        for (unsigned j = 0; j < 4; ++j) if (!std::isfinite(row[j])) return false;
        const bool idle = row[0] == 0 && row[1] == 0 && row[2] == 0 && row[3] == 0;
        if (idle) continue;
        if (!servos[i] && row[0] > 0 && row[1] == 0 && row[2] == 0 && row[3] == 0) throttle = true;
        else if (servos[i] && row[0] == 0 && row[1] == 0 && row[2] == 0 && row[3] != 0) steering = true;
        else return false;
    }
    return throttle && steering;
}

// Uses the original throttle on every call; no feedback into RC channel state.
// The bounded monotonic queue gives an exact 250 ms maximum at the 250 Hz loop.
class DriftAssist {
public:
    void reset() { _head = _count = 0; _reduction = 0; }
    float reduction() const { return _reduction; }
    float update(float throttle, float targetDps, float measuredDps, float dt,
        uint32_t nowUs, bool requested, const DriftAssistSettings &s)
    {
        if (!std::isfinite(throttle) || !std::isfinite(targetDps) || !std::isfinite(measuredDps) ||
            !std::isfinite(dt) || dt <= 0 || dt > 0.1f || !driftSettingsValid(s)) {
            reset();
            return std::isfinite(throttle) ? throttle : 0.0f;
        }
        if (throttle <= 0) { reset(); return 0; }
        const float target = fabsf(targetDps);
        while (_count && (uint32_t)(nowUs - _samples[_head].timeUs) >= 250000U) {
            _head = (_head + 1) % CAPACITY;
            --_count;
        }
        while (_count && _samples[(_head + _count - 1) % CAPACITY].value <= target) --_count;
        if (_count == CAPACITY) { _head = (_head + 1) % CAPACITY; --_count; }
        _samples[(_head + _count) % CAPACITY] = {nowUs, target};
        ++_count;
        const float excess = fmaxf(0.0f, fabsf(measuredDps) - _samples[_head].value - s.thresholdDps);
        const float desired = requested ? s.maxReductionPercent * 0.01f * fminf(1.0f, excess / s.rampDps) : 0;
        const float tau = (desired > _reduction ? s.attackMs : s.recoveryMs) * 0.001f;
        _reduction += (desired - _reduction) * (1.0f - expf(-dt / tau));
        if (_reduction < 0.00001f) _reduction = 0;
        return throttle * (1.0f - _reduction);
    }
private:
    struct Sample { uint32_t timeUs; float value; };
    static constexpr unsigned CAPACITY = 128;
    Sample _samples[CAPACITY] = {};
    unsigned _head = 0, _count = 0;
    float _reduction = 0;
};
