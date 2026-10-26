// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include "communication/OdomI2cLog.hpp"
#include "sensor_state.hpp"
#include "tap/architecture/clock.hpp"
namespace hosted {
class Pods {
public:
    Pods() { log.enabled = std::getenv("MCB_ODOM_I2C_LOG") != nullptr; }
    float offsetX = 0, offsetY = 0;
    float getX() { return sensors.podX + offsetX; }
    float getY() { return sensors.podY + offsetY; }
    float getXVel() { return sensors.podVx; }
    float getYVel() { return sensors.podVy; }
    // One successful Pico read per call; the firmware fixture enables it with MCB_ODOM_I2C_LOG.
    void poll() {
        const float payload[4] = {sensors.podX, sensors.podY, sensors.podVx, sensors.podVy};
        log.attempt(tap::arch::clock::getTimeMicroseconds(), tap::arch::clock::getTimeMilliseconds(), 0, payload, getX(), getY());
    }
    communication::OdomI2cLog log;
};
class Encoder {
public:
    float getAngle() { return sensors.encoder; }
    uint16_t getRawAngle() { return getAngle() * 16384 / (2 * M_PI); }
    void setAngleInvertedTrue() {}
    void setAngleInvertedFalse() {}
};
}
