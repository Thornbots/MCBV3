#pragma once

#include "Ballistics.hpp"

namespace aim {
constexpr float SHOT_GATING_RAD_S = 3.14159265358979f / 2.0f;
struct AimTuning {
    float replan_period_s = 0.005f;
    float firmware_latency_s = 0.020f;
    bool chase = true;
    float shot_gating_rad_s = SHOT_GATING_RAD_S;
    float chase_settle_s = 0.025f;
    float chase_margin_s = 0.025f;
    float panel_switch_margin_rad = 0.3f;
    float spin_enter_rad_s = 3.0f;
    float spin_exit_rad_s = 2.0f;
    float v_muzzle_mps = 22.0f;
    BallisticModel ballistics{};
};
}  // namespace aim
