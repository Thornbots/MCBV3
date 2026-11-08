// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
namespace hosted {
struct Sensors {
    float podX = 0, podY = 0, podVx = 0, podVy = 0;
    float yaw = 0, yawRate = 0, encoder = 0;
    float jointYaw = 0, jointYawRate = 0, jointPitch = 0, jointPitchRate = 0;
};
inline Sensors sensors;
}
