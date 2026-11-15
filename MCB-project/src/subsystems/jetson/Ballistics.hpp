#pragma once

#include <cmath>
#include <optional>

namespace aim {
// Pivot-relative forward/up bore offsets, positive-down pitch and horizontal drag.
struct BallisticModel {
    float speed = 22.0f;
    float drag = 0.02f;
    float bias = 0.042f;
    float bore_forward = 0.173f;
    float bore_up = 0.0f;
    float bore_right = -0.0019f;
    float gravity = 9.80665f;
};

struct HeightSample {
    float height, time, pitch_derivative, time_pitch, time_range, height_range;
};

inline std::optional<HeightSample> ballistic_height(float range, float pitch, const BallisticModel& m) {
    const float s = std::sin(pitch), c = std::cos(pitch);
    const float forward = m.bore_forward * c + m.bore_up * s;
    const float up = -m.bore_forward * s + m.bore_up * c;
    const float vh = m.speed * std::cos(pitch + m.bias);
    const float vz = -m.speed * std::sin(pitch + m.bias);
    const float d = range - forward;
    if (!(d > 0 && vh > 0 && m.drag >= 0 && m.gravity > 0)) return std::nullopt;
    const float e1 = std::expm1(m.drag * d);
    const float time = m.drag > 0 ? e1 / (m.drag * vh) : d / vh;
    const float tr = (e1 + 1) / vh;
    const float tp = -tr * up - time * vz / vh;
    const float vertical = vz - m.gravity * time;
    const float height = up + vz * time - 0.5f * m.gravity * time * time;
    const float hp = -forward - vh * time + vertical * tp;
    if (!std::isfinite(height) || !std::isfinite(hp) || !std::isfinite(time)) return std::nullopt;
    return HeightSample{height, time, hp, tp, tr, vertical * tr};
}

struct BallisticSolution {
    float pitch, time;
    float pitch_range, pitch_height, time_range, time_height;
    unsigned corrections;
};

inline std::optional<BallisticSolution> solve_ballistic(float range, float height, const BallisticModel& m, std::optional<float> seed = std::nullopt) {
    if (!std::isfinite(range) || !std::isfinite(height) || !(m.speed > 0)) return std::nullopt;
    const float v2 = m.speed * m.speed;
    const float disc = v2 * v2 - m.gravity * (m.gravity * range * range + 2 * height * v2);
    if (!(range > 0 && disc >= 0)) return std::nullopt;
    // Rationalized low arc avoids subtracting nearly equal squares near zero height.
    float p = seed ? *seed : -std::atan2(m.gravity * range * range + 2 * height * v2, range * (v2 + std::sqrt(disc))) - m.bias;
    for (unsigned i = 0; i <= 3; ++i) {
        const auto h = ballistic_height(range, p, m);
        if (!h || std::abs(h->pitch_derivative) < 1e-5f) return std::nullopt;
        const float residual = h->height - height;
        if (std::abs(residual) <= 0.0005f) {
            const float ph = 1 / h->pitch_derivative;
            const float pr = -h->height_range * ph;
            return BallisticSolution{p, h->time, pr, ph, h->time_range + h->time_pitch * pr, h->time_pitch * ph, i};
        }
        if (i == 3) break;
        p -= residual / h->pitch_derivative;
    }
    return std::nullopt;
}
}  // namespace aim
