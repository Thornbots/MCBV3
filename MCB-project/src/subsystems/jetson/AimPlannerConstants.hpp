#pragma once

#include <cstdint>

// Tuned values for AimPlanner, moved from the Jetson's point_to_cv_target defaults
// (same numbers, same meaning). The README's "MCBV3 aiming" section owns the rationale.
// Seconds, metres and radians unless named otherwise. No hardware dependencies.
namespace aim
{
struct AimTuning
{
    float replan_period_s = 0.025f;       // Jetson tick_s (40 Hz): each aim is held this long
    float gimbal_lag_s = 0.05f;           // how far the gimbal trails the aim point
    float firmware_latency_s = 0.020f;    // indexer request to muzzle exit (FIRING_LATENCY_TIME)
    float min_fire_delay_s = 0.040f;      // exit time never modeled under this
    bool lead = true;
    int tof_iterations = 3;
    bool chase = true;                    // false: shotgating while spinning
    float chase_settle_s = 0.025f;        // chase only
    float chase_margin_s = 0.025f;        // chase only
    float panel_switch_margin_rad = 0.3f;
    float spin_enter_rad_s = 3.0f;        // |yaw_rate| above this: spin mode
    float spin_exit_rad_s = 2.0f;         // back to panel mode below this
    float v_muzzle_mps = 22.0f;           // overridden with initialShotVelocity
    float fire_confidence = 0.5f;         // compared with confidence / 255
    uint32_t fire_lock_ms = 1000;         // same-track dwell before firing
    uint32_t fire_lock_grace_ms = 300;    // ineligible gap that keeps the dwell
    float fire_stationary_speed_mps = 0.1f;  // below this, outside spin mode, skips the dwell
    float fire_max_detection_age_s = 0.15f;
    float target_timeout_s = 0.5f;        // a held state aims until its age reaches this
};
}  // namespace aim
