#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

#include "AimPlannerConstants.hpp"

namespace aim {
constexpr float PI_F = 3.14159265358979f;
constexpr float QUARTER_TURN = PI_F / 2.0f;
struct Vec3 {
    float x = 0, y = 0, z = 0;
};
inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(const Vec3& a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
float norm(const Vec3& a);
float py_mod(float a, float b);
float wrap_to_pi(float a);
int choose_panel(float yaw, float bearing, std::optional<float> held_yaw, float margin_rad);

struct TargetInput {
    Vec3 center, velocity, acceleration, panel;
    float yaw = 0, yaw_rate = 0;
    std::array<float, 2> radius{}, z_offset{};
    float state_age_s = 0, detection_age_s = 0;
    uint32_t track_id = 0;
    float confidence = 0;  // diagnostic only
    bool valid = false, clear = false;
};

struct Panel {
    Vec3 center, velocity, normal, width, height, normal_rate;
};
Panel panel_at(const TargetInput& in, int k, float time);
bool usable(const TargetInput& in);
inline bool inside_panel(float width, float height) { return std::abs(width) <= 0.0675f && std::abs(height) <= 0.0625f; }

// World yaw, positive-down pitch, and physical angular rates in those frames.
struct GimbalMotion {
    float yaw = 0, pitch = 0, yaw_rate = 0, pitch_rate = 0;
};
struct Plan {
    Vec3 aim, aim_rate;
    float yaw = 0, pitch = 0, yaw_rate = 0, pitch_rate = 0, flight_time = 0;
    float time_rate = 0;
    int panel = 0;
    bool valid = false, spinning = false;
};
struct Impact {
    float time, width, height;
    bool inside;
};

// Current-measurement gate; no cached or latched firing permission.
std::optional<Impact> predict_impact(const TargetInput& in, const Plan& plan, const Vec3& pivot_now, const Vec3& shooter_vel, const GimbalMotion& measured, const AimTuning& tuning);
bool shot_window(const TargetInput& in, int panel, float flight_time, const Vec3& pivot_now, const Vec3& shooter_vel, bool spinning, const AimTuning& tuning);

class AimPlanner {
public:
    explicit AimPlanner(const AimTuning& tuning = {}) : t(tuning) {}
    const AimTuning& tuning() const { return t; }
    void reset();
    void observe(uint32_t now_ms, const TargetInput& in);
    Plan plan(uint32_t now_ms, const TargetInput& in, const Vec3& pivot_now, const Vec3& shooter_vel);

private:
    AimTuning t;
    bool spinning_ = false, has_track_ = false;
    uint32_t track_id_ = 0;
    std::optional<float> held_panel_yaw_, pitch_seed_, time_seed_;
};

// Shared board/stand-in lifecycle and 1 ms control seam. No queued shot survives a tick.
class AimControl {
public:
    enum class State { WAITING, TRACKING };
    explicit AimControl(const AimTuning& tuning = {}) : planner_(tuning) {}
    void reset();
    bool observe(uint32_t receipt_ms, const TargetInput& in);
    Plan tick(uint32_t now_ms, const Vec3& pivot_now, const Vec3& shooter_vel);
    bool can_fire(uint32_t now_ms, const Vec3& pivot_now, const Vec3& shooter_vel, const GimbalMotion& measured) const;
    State state() const { return state_; }
    bool replanned() const { return replanned_; }
    const AimTuning& tuning() const { return planner_.tuning(); }

private:
    TargetInput aged(uint32_t now_ms) const;
    AimPlanner planner_;
    State state_ = State::WAITING;
    TargetInput held_{};
    Plan solved_{};
    uint32_t received_ms_ = 0, solved_ms_ = 0;
    bool dirty_ = false, replanned_ = false;
};
}  // namespace aim
