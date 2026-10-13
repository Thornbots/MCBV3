// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cmath>

namespace subsystems {
// Absolute CV points arrive more slowly than the control loop. Estimate their
// continuous angular rate without a one-tick derivative kick or a wrap spike.
class AngleSetpointRate {
public:
    float update(float angle, float dt) {
        if (dt <= 0) return 0;
        if (!initialized) { previous = angle; initialized = true; return 0; }
        const float change = std::remainder(angle - previous, 6.28318530718f);
        previous = angle;
        if (std::abs(change) > .25f) rate = 0; // target handoff, not angular velocity
        else rate += dt / (.025f + dt) * (change / dt - rate);
        return rate;
    }
    void reset() { initialized = false; rate = 0; }
private:
    bool initialized = false;
    float previous = 0, rate = 0;
};
} // namespace subsystems
