// Copyright 2026 Thornbots
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cstdint>

namespace subsystems {

inline constexpr uint32_t kHeatPer17mm = 10;
inline constexpr uint32_t kHeatPer42mm = 100;
inline constexpr uint32_t kHeatTickMs = 100;
inline constexpr uint32_t kSentryShotIntervalMs = 100;
inline constexpr uint32_t kSentryEstimatedRounds = 2;

class LauncherHeat {
public:
    void reset(uint32_t nowMs) {
        lastTickMs = nowMs;
        heat = 0;
        remainder = 0;
    }

    void cool(uint32_t nowMs, uint32_t coolingPerSecond) {
        const uint32_t ticks = (nowMs - lastTickMs) / kHeatTickMs;
        if (ticks == 0) return;
        lastTickMs += ticks * kHeatTickMs;
        const uint64_t tenths = uint64_t{ticks} * coolingPerSecond + remainder;
        heat -= std::min<uint64_t>(heat, tenths / 10);
        remainder = tenths % 10;
    }

    bool canCharge(uint32_t limit, uint32_t charge) const {
        return heat + charge <= limit;
    }

    void charge(uint32_t amount) { heat += amount; }
    uint32_t value() const { return heat; }

private:
    uint32_t lastTickMs = 0;
    uint32_t heat = 0;
    uint32_t remainder = 0;
};

}  // namespace subsystems
