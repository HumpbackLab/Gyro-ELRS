#pragma once

#include <stdint.h>

// One complete hopping cycle plus the existing 10% synchronization margin.
inline uint32_t rxScanWindowMs(uint32_t channels, uint8_t hopInterval, uint32_t packetIntervalUs)
{
    return 11U * channels * hopInterval * packetIntervalUs / 10000U;
}

template <uint8_t Capacity>
class RxScanOrder
{
public:
    // A zero window excludes an unsupported mode. Ties retain RF table order.
    template <typename Window>
    uint8_t begin(uint8_t preferred, Window window)
    {
        count = 0;
        for (uint8_t rate = 0; rate < Capacity; ++rate)
        {
            const uint32_t duration = window(rate);
            if (!duration) continue;
            uint8_t pos = count;
            while (pos && durations[pos - 1] > duration)
            {
                rates[pos] = rates[pos - 1];
                durations[pos] = durations[pos - 1];
                --pos;
            }
            rates[pos] = rate;
            durations[pos] = duration;
            ++count;
        }
        // Keep a valid saved rate first, then visit every other mode by cost.
        for (uint8_t pos = 0; pos < count; ++pos)
        {
            if (rates[pos] != preferred) continue;
            for (; pos; --pos) rates[pos] = rates[pos - 1];
            rates[0] = preferred;
            break;
        }
        cursor = 0;
        return count ? rates[0] : 0;
    }

    uint8_t next()
    {
        if (!count) return 0;
        cursor = (cursor + 1) % count;
        return rates[cursor];
    }

    // After binding, the next scan starts with the first supported mode.
    void restart() { cursor = count ? count - 1 : 0; }

private:
    uint8_t rates[Capacity] = {};
    uint32_t durations[Capacity] = {};
    uint8_t count = 0;
    uint8_t cursor = 0;
};
