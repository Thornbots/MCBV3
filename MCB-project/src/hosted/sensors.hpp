// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include "communication/OdomI2cLog.hpp"
#include "sensor_state.hpp"
#include "tap/architecture/clock.hpp"
namespace hosted {
class Pods {
public:
    Pods() {
        log.enabled = std::getenv("MCB_ODOM_I2C_LOG") != nullptr;
        // MCB_POD_FAULT=nack@MS: one failed read at MS, then the poll latches (no more reads).
        // MCB_POD_FAULT=stuck@MS: from MS the read never completes (no more attempts).
        if (const char* fault = std::getenv("MCB_POD_FAULT")) {
            nackFault = fault[0] == 'n';
            if (const char* at = std::strchr(fault, '@')) faultMs = std::strtoul(at + 1, nullptr, 10);
        }
    }
    float offsetX = 0, offsetY = 0;
    float getX() { return sensors.podX + offsetX; }
    float getY() { return sensors.podY + offsetY; }
    float getXVel() { return sensors.podVx; }
    float getYVel() { return sensors.podVy; }
    // One successful Pico read per call; the firmware fixture enables it with MCB_ODOM_I2C_LOG.
    void poll() {
        const uint32_t nowMs = tap::arch::clock::getTimeMilliseconds();
        if (latched) return;
        const bool fail = nowMs >= faultMs;
        const float payload[4] = {sensors.podX, sensors.podY, sensors.podVx, sensors.podVy};
        if (fail) latched = true;
        if (fail && !nackFault) return;
        log.attempt(tap::arch::clock::getTimeMicroseconds(), nowMs, fail ? 2 : 0, payload, getX(), getY());
    }
    bool isFresh(uint32_t nowMs, uint32_t maxAgeMs) const { return log.succeededWithin(nowMs, maxAgeMs); }
    communication::OdomI2cLog log;

private:
    uint32_t faultMs = UINT32_MAX;
    bool nackFault = false, latched = false;
};
class Encoder {
public:
    float getAngle() { return sensors.encoder; }
    uint16_t getRawAngle() { return getAngle() * 16384 / (2 * M_PI); }
    void setAngleInvertedTrue() {}
    void setAngleInvertedFalse() {}
};
}
