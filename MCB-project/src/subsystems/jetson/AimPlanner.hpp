#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "AimPlannerConstants.hpp"

// Shot planner ported from the Jetson's point_to_cv_target_core (float, no hardware
// dependencies). Semantics match it; see the README's "MCBV3 aiming" section.
namespace aim
{
constexpr float PI_F = 3.14159265358979f;
constexpr float QUARTER_TURN = PI_F / 2.0f;

struct Vec3
{
    float x = 0, y = 0, z = 0;
};
inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(const Vec3& a, float s) { return {a.x * s, a.y * s, a.z * s}; }
float norm(const Vec3& a);

// Python-style floor modulo: the result takes the divisor's sign.
float py_mod(float a, float b);
// Wrap an angle to [-pi, pi).
float wrap_to_pi(float a);

// [xc, yc, zc, vx, vy, vz, yaw, w] in odom at its stamp.
using ArmorState = std::array<float, 8>;

struct ShotOptions
{
    float gimbal_lag_s = 0.05f;
    float firmware_latency_s = 0.020f;  // MCB indexer request to chamber exit
    float min_fire_delay_s = 0.040f;    // allow the gimbal to acquire each aim point
    bool lead = true;
    int iterations = 3;
    Vec3 shooter_vel{};
    std::optional<float> chase_settle_s;  // set: chase the facing panel while spinning
    float chase_margin_s = 0.0f;
    Vec3 accel{};  // the center's
    // Not spinning: Shot::panel_yaw from last tick, kept until another panel
    // faces us panel_switch_margin_rad more squarely.
    std::optional<float> held_panel_yaw;
    float panel_switch_margin_rad = 0.0f;
};

struct Shot
{
    Vec3 aim_pos;                  // odom point a gun leaving from the shooter points through
    std::optional<float> delay_s;  // fire after this long, or nullopt for no shot this tick
    std::optional<float> panel_yaw = std::nullopt;  // not spinning: led panel's normal yaw
};

// Panel 0-3 to lead on a robot that isn't spinning: the most face-on, except the
// panel nearest held_yaw stays while no other faces us margin_rad more squarely.
int choose_panel(float yaw, float bearing, std::optional<float> held_yaw, float margin_rad);

// Aim point and fire delay against a TargetState armor model, age_s old.
// radius, z_offset: per panel pair. Modes: panel lead (not spinning), shotgating or
// chase (spinning), no lead. aim_pos is the intercept less our motion over the flight.
Shot plan_shot(const ArmorState& state, const std::array<float, 2>& radius,
               const std::array<float, 2>& z_offset, float age_s, const Vec3& shooter_pos,
               float v_muzzle, bool spinning, float tick_s, const ShotOptions& options = {});

// Fire decision to chamber exit: the MCB's latency, but never under the floor.
float fire_exit_s(float firmware_latency_s, float min_fire_delay_s);

// Jetson's former wire delay: alignment wait plus exit, rounded to ms, clamped to uint16.
int wire_fire_delay_ms(float alignment_s, float exit_s);

// Same-track dwell on the millisecond clock (wrap-safe), including brief ineligible gaps.
// The caller supplies convergence/confidence/freshness; shot windows are separate.
class FireLock
{
public:
    bool update(uint32_t now_ms, uint32_t track_id, bool eligible, bool spinning,
                float speed_mps, uint32_t lock_ms, float stationary_speed_mps,
                uint32_t grace_ms);
    void reset()
    {
        has_since_ = false;
        has_ineligible_ = false;
    }

private:
    bool has_since_ = false;
    bool has_ineligible_ = false;
    uint32_t since_ms_ = 0;
    uint32_t ineligible_since_ms_ = 0;
    uint32_t last_ms_ = 0;
    uint32_t track_id_ = 0;
};

// One TargetState frame in SI units, with ages valid at the time it is handed over.
struct TargetInput
{
    Vec3 center, velocity, acceleration, panel;
    float yaw = 0, yaw_rate = 0;
    std::array<float, 2> radius{}, z_offset{};
    float state_age_s = 0;
    float detection_age_s = 0;
    uint32_t track_id = 0;
    float confidence = 0;  // 0..1
    bool valid = false;
};

struct Plan
{
    Vec3 aim;
    bool fire = false;
    int request_delay_ms = 0;  // from the plan time to the indexing request (fire only)
    bool spinning = false;
};

// Per-target planner state: spin hysteresis, held panel and fire lock.
class AimPlanner
{
public:
    explicit AimPlanner(const AimTuning& tuning = {}) : t(tuning) {}
    const AimTuning& tuning() const { return t; }
    void reset();
    // A new frame arrived: track change, invalid and spin-entry clear the panel hold.
    void observe(uint32_t now_ms, const TargetInput& in);
    // Replan against the held frame, whose ages are current.
    Plan plan(uint32_t now_ms, const TargetInput& in, const Vec3& shooter_pos,
              const Vec3& shooter_vel);

private:
    bool updateLock(uint32_t now_ms, const TargetInput& in);

    AimTuning t;
    bool spinning_ = false;
    bool has_track_ = false;
    uint32_t track_id_ = 0;
    std::optional<float> held_panel_yaw_;
    FireLock lock_;
};
}  // namespace aim
