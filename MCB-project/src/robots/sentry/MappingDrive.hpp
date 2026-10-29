// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include "util/Pose2d.hpp"

namespace robots {
// Chassis-local convention: x right, y forward; angular velocity is rad/s.
inline constexpr float MAPPING_MAX_TRANSLATION_MPS = 0.4f;
inline constexpr float MAPPING_MAX_ROTATION_RADPS = 0.5f;

inline Pose2d mappingDrive(float right, float forward, float rotation, bool connected) {
    if (!connected || !std::isfinite(right) || !std::isfinite(forward) || !std::isfinite(rotation))
        return Pose2d(0, 0, 0);
    right = std::clamp(right, -1.0f, 1.0f);
    forward = std::clamp(forward, -1.0f, 1.0f);
    const float scale = MAPPING_MAX_TRANSLATION_MPS / std::max(1.0f, std::hypot(right, forward));
    return Pose2d(right * scale, forward * scale,
                  std::clamp(rotation, -1.0f, 1.0f) * MAPPING_MAX_ROTATION_RADPS);
}
}  // namespace robots
