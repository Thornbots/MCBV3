// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cmath>
#include <cstdint>
#include "sensor_state.hpp"
#include "tap/architecture/clock.hpp"
namespace hosted {
class Pods {
public:
    void poll() { lastSuccessMs = tap::arch::clock::getTimeMilliseconds(); everSucceeded = true; }
    bool isFresh(uint32_t nowMs, uint32_t maxAgeMs) const {
        return everSucceeded && nowMs - lastSuccessMs < maxAgeMs;
    }
    float offsetX = 0, offsetY = 0;
    float getX() { return sensors.podX + offsetX; }
    float getY() { return sensors.podY + offsetY; }
    float getXVel() { return sensors.podVx; }
    float getYVel() { return sensors.podVy; }
private:
    uint32_t lastSuccessMs = 0;
    bool everSucceeded = false;
};
class Encoder {
public:
    float getAngle() { return sensors.encoder; }
    uint16_t getRawAngle() { return getAngle() * 16384 / (2 * M_PI); }
    void setAngleInvertedTrue() {}
    void setAngleInvertedFalse() {}
};
}
