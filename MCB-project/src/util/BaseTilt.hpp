// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <algorithm>
#include <cmath>

// Barrel tilt from a yaw-link gyro: world up is propagated in IMU axes
// (u' = -w x u), projected on the zero-pitch barrel direction and high-passed,
// so static tilt and gyro drift stay in the encoder frame. README.md#base-tilt.
class BaseTilt {
public:
    BaseTilt(const float (&forward)[3], float pitchUpSign, float tauS)
        : f{forward[0], forward[1], forward[2]}, sign(pitchUpSign), tau(tauS) {}

    void update(float gx, float gy, float gz, float dt) {
        const float u0 = up[0], u1 = up[1], u2 = up[2];
        up[0] -= (gy * u2 - gz * u1) * dt;
        up[1] -= (gz * u0 - gx * u2) * dt;
        up[2] -= (gx * u1 - gy * u0) * dt;
        const float n = std::sqrt(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
        for (float& c : up) c /= n;
        const float tilt =
            sign * std::asin(std::clamp(up[0] * f[0] + up[1] * f[1] + up[2] * f[2], -1.0f, 1.0f));
        if (!started) {
            mean = tilt;
            started = true;
        }
        mean += (tilt - mean) * dt / tau;
        const float next = tilt - mean;
        rate = (next - angle) / dt;
        angle = next;
    }

    float angle = 0;  // pitch-encoder sense, rad
    float rate = 0;   // rad/s

private:
    float f[3];
    float sign, tau;
    float up[3] = {0, 0, 1};
    float mean = 0;
    bool started = false;
};
