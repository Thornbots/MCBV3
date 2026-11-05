#include "AimPlanner.hpp"

#include <algorithm>
#include <cmath>

namespace aim
{
float norm(const Vec3& a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }

float py_mod(float a, float b)
{
    float r = std::fmod(a, b);
    if (r != 0.0f) {
        if ((b < 0.0f) != (r < 0.0f)) r += b;
    } else {
        r = std::copysign(0.0f, b);
    }
    return r;
}

float wrap_to_pi(float a) { return py_mod(a + PI_F, 2.0f * PI_F) - PI_F; }

namespace
{
// Python round(): half to even, then % 4 as a non-negative index.
int quarter_index(float turns)
{
    if (!std::isfinite(turns)) return 0;
    const int64_t k = static_cast<int64_t>(std::nearbyint(turns));
    return static_cast<int>(((k % 4) + 4) % 4);
}
}  // namespace

int choose_panel(float yaw, float bearing, std::optional<float> held_yaw, float margin_rad)
{
    std::array<float, 4> off{};
    int best = 0;
    for (int k = 0; k < 4; ++k) {
        off[k] = std::abs(wrap_to_pi(yaw + k * QUARTER_TURN - bearing));
        if (off[k] < off[best]) best = k;
    }
    if (!held_yaw) return best;
    const int held = quarter_index((*held_yaw - yaw) / QUARTER_TURN);
    return off[held] <= off[best] + margin_rad ? held : best;
}

float fire_exit_s(float firmware_latency_s, float min_fire_delay_s)
{
    return std::max(firmware_latency_s, min_fire_delay_s);
}

int wire_fire_delay_ms(float alignment_s, float exit_s)
{
    const float ms = std::nearbyint((std::max(alignment_s, 0.0f) + exit_s) * 1000.0f);
    return static_cast<int>(std::clamp(ms, 0.0f, 65535.0f));
}

Shot plan_shot(const ArmorState& state, const std::array<float, 2>& radius,
               const std::array<float, 2>& z_offset, float age_s, const Vec3& shooter_pos,
               float v_muzzle, bool spinning, float tick_s, const ShotOptions& options)
{
    const float xc = state[0], yc = state[1], zc = state[2];
    const Vec3 v{state[3], state[4], state[5]};
    const float yaw = state[6], w = state[7];
    const Vec3 acc = options.accel;
    const Vec3 shooter_vel = options.shooter_vel;
    const int iterations = options.iterations;
    const float aim_horizon_s = age_s + options.gimbal_lag_s + tick_s / 2.0f;
    const float fire_horizon_s =
        age_s + fire_exit_s(options.firmware_latency_s, options.min_fire_delay_s);

    auto center = [&](float t) -> Vec3 {
        const float h = 0.5f * t * t;
        return {xc + v.x * t + acc.x * h, yc + v.y * t + acc.y * h, zc + v.z * t + acc.z * h};
    };
    auto shooter = [&](float t) -> Vec3 { return shooter_pos + shooter_vel * t; };
    auto bearing = [&](float t) {
        const Vec3 c = center(t), s = shooter(t);
        return std::atan2(s.y - c.y, s.x - c.x);
    };
    auto facing = [&](float t) {
        return quarter_index((bearing(t) - (yaw + w * t)) / QUARTER_TURN);
    };
    auto panel = [&](int k, float t) -> Vec3 {
        const Vec3 c = center(t);
        const float yaw_k = yaw + w * t + k * QUARTER_TURN, r = radius[k % 2];
        return {c.x + r * std::cos(yaw_k), c.y + r * std::sin(yaw_k), c.z + z_offset[k % 2]};
    };
    auto facing_panel = [&](float t) -> Vec3 { return panel(facing(t), t); };
    auto on_line = [&](float r, float dz) {
        return [&, r, dz](float t) -> Vec3 {
            const Vec3 c = center(t);
            const float b = bearing(t);
            return {c.x + r * std::cos(b), c.y + r * std::sin(b), c.z + dz};
        };
    };

    struct Intercept
    {
        Vec3 aim_pos;
        float t_flight;
    };
    // Fixed point on the target's true path: t <- |path(h + t) - shooter(h + t)| / v_muzzle.
    auto intercept = [&](const auto& path) -> Intercept {
        float t = 0.0f;
        for (int i = 0; i < std::max(1, iterations); ++i) {
            t = v_muzzle > 0.0f
                    ? norm(path(aim_horizon_s + t) - shooter(aim_horizon_s + t)) / v_muzzle
                    : 0.0f;
        }
        const Vec3 hit = path(aim_horizon_s + t);
        return {hit - shooter_vel * t, t};
    };

    if (spinning && !options.lead) return {facing_panel(0.0f), 0.0f, std::nullopt};
    if (!spinning) {
        // Choose near impact, a flight past the aim horizon.
        const float t_sel =
            options.lead
                ? aim_horizon_s + norm(center(aim_horizon_s) - shooter(aim_horizon_s)) /
                                      std::max(v_muzzle, 1e-6f)
                : 0.0f;
        const int k = choose_panel(yaw + w * t_sel, bearing(t_sel), options.held_panel_yaw,
                                   options.panel_switch_margin_rad);
        const float panel_yaw = wrap_to_pi(yaw + w * t_sel + k * QUARTER_TURN);
        const auto chosen = [&](float t) -> Vec3 { return panel(k, t); };
        if (!options.lead) return {chosen(0.0f), 0.0f, panel_yaw};
        const auto hit = intercept(chosen);
        return {hit.aim_pos, 0.0f,
                wrap_to_pi(yaw + w * (aim_horizon_s + hit.t_flight) + k * QUARTER_TURN)};
    }
    if (options.chase_settle_s) {
        const Intercept hit = intercept(facing_panel);
        // Leave mid-hold of whichever aim is current then: that aim was solved for this exit.
        const float delay = py_mod(aim_horizon_s - fire_horizon_s, tick_s);
        const float t_impact = fire_horizon_s + delay + hit.t_flight;
        const int k = facing(t_impact);
        float theta = yaw + w * t_impact + k * QUARTER_TURN - bearing(t_impact);
        theta = py_mod(theta + QUARTER_TURN / 2.0f, QUARTER_TURN);
        const float since =
            w > 0.0f ? theta / std::abs(w) : (QUARTER_TURN - theta) / std::abs(w);
        const float until = QUARTER_TURN / std::abs(w) - since;
        const bool fire = since >= *options.chase_settle_s && until >= options.chase_margin_s;
        return {hit.aim_pos, fire ? std::optional<float>(delay) : std::nullopt, std::nullopt};
    }

    const float t_flight = intercept(on_line(0.5f * (radius[0] + radius[1]), 0.0f)).t_flight;
    if (std::abs(w) < 1e-6f) {
        return {intercept(on_line(radius[0], z_offset[0])).aim_pos, std::nullopt, std::nullopt};
    }

    auto to_alignment = [&](float t_impact) {
        // Time from t_impact until a panel normal points along the line to the shooter.
        const float phase = py_mod(yaw + w * t_impact - bearing(t_impact), QUARTER_TURN);
        return (w > 0.0f ? py_mod(QUARTER_TURN - phase, QUARTER_TURN) : phase) / std::abs(w);
    };

    // Hold the pair until the last shot at it has left the muzzle, then switch.
    const float t_pair = age_s + t_flight;
    const int k = facing(t_pair + to_alignment(t_pair));
    const Intercept hit = intercept(on_line(radius[k % 2], z_offset[k % 2]));
    const float delay = to_alignment(fire_horizon_s + t_flight);
    return {hit.aim_pos, delay < tick_s ? std::optional<float>(delay) : std::nullopt,
            std::nullopt};
}

bool FireLock::update(uint32_t now_ms, uint32_t track_id, bool eligible, bool spinning,
                      float speed_mps, uint32_t lock_ms, float stationary_speed_mps,
                      uint32_t grace_ms)
{
    const bool rewound = static_cast<int32_t>(now_ms - last_ms_) < 0;
    const bool gap_too_long =
        has_ineligible_ && static_cast<int32_t>(now_ms - ineligible_since_ms_) >
                               static_cast<int32_t>(grace_ms);
    if (track_id != track_id_ || rewound || gap_too_long) reset();
    track_id_ = track_id;
    last_ms_ = now_ms;
    if (!eligible) {
        if (!has_ineligible_) {
            has_ineligible_ = true;
            ineligible_since_ms_ = now_ms;
        }
        return false;
    }
    has_ineligible_ = false;
    if (!has_since_) {
        has_since_ = true;
        since_ms_ = now_ms;
    }
    return lock_ms == 0 || (!spinning && speed_mps < stationary_speed_mps) ||
           static_cast<int32_t>(now_ms - since_ms_) >= static_cast<int32_t>(lock_ms);
}

void AimPlanner::reset()
{
    spinning_ = false;
    has_track_ = false;
    held_panel_yaw_.reset();
    lock_.reset();
}

bool AimPlanner::updateLock(uint32_t now_ms, const TargetInput& in)
{
    const bool eligible = in.valid && in.confidence >= t.fire_confidence &&
                          in.detection_age_s >= 0.0f &&
                          in.detection_age_s <= t.fire_max_detection_age_s;
    return lock_.update(now_ms, in.track_id, eligible, spinning_, norm(in.velocity),
                        t.fire_lock_ms, t.fire_stationary_speed_mps, t.fire_lock_grace_ms);
}

void AimPlanner::observe(uint32_t now_ms, const TargetInput& in)
{
    if (!has_track_ || in.track_id != track_id_) {
        spinning_ = false;
        held_panel_yaw_.reset();
    }
    if (!in.valid) {
        spinning_ = false;
        held_panel_yaw_.reset();
    }
    if (std::abs(in.yaw_rate) > (spinning_ ? t.spin_exit_rad_s : t.spin_enter_rad_s))
        held_panel_yaw_.reset();
    has_track_ = true;
    track_id_ = in.track_id;
    updateLock(now_ms, in);
}

Plan AimPlanner::plan(uint32_t now_ms, const TargetInput& in, const Vec3& shooter_pos,
                      const Vec3& shooter_vel)
{
    if (!in.valid) {
        // Unconverged: aim at the measured panel, no lead, no fire.
        spinning_ = false;
        held_panel_yaw_.reset();
        updateLock(now_ms, in);
        return {in.panel, false, 0, false};
    }
    spinning_ = std::abs(in.yaw_rate) > (spinning_ ? t.spin_exit_rad_s : t.spin_enter_rad_s);
    const ArmorState armor = {in.center.x, in.center.y, in.center.z, in.velocity.x,
                              in.velocity.y, in.velocity.z, in.yaw, in.yaw_rate};
    ShotOptions o;
    o.gimbal_lag_s = t.gimbal_lag_s;
    o.firmware_latency_s = t.firmware_latency_s;
    o.min_fire_delay_s = t.min_fire_delay_s;
    o.lead = t.lead;
    o.iterations = t.tof_iterations;
    o.shooter_vel = shooter_vel;
    if (t.chase) o.chase_settle_s = t.chase_settle_s;
    o.chase_margin_s = t.chase_margin_s;
    o.accel = in.acceleration;
    o.held_panel_yaw = spinning_ ? std::nullopt : held_panel_yaw_;
    o.panel_switch_margin_rad = t.panel_switch_margin_rad;
    const Shot shot = plan_shot(armor, in.radius, in.z_offset, in.state_age_s, shooter_pos,
                                t.v_muzzle_mps, spinning_, t.replan_period_s, o);
    held_panel_yaw_ = shot.panel_yaw;

    const bool locked = updateLock(now_ms, in);
    Plan out{shot.aim_pos, false, 0, spinning_};
    const bool age_ok = in.detection_age_s >= 0.0f && in.detection_age_s <= t.fire_max_detection_age_s;
    if (shot.delay_s && locked && age_ok) {
        const int latency_ms = static_cast<int>(std::nearbyint(t.firmware_latency_s * 1000.0f));
        const int wire_ms = wire_fire_delay_ms(
            *shot.delay_s, fire_exit_s(t.firmware_latency_s, t.min_fire_delay_s));
        out.fire = true;
        out.request_delay_ms = std::max(0, wire_ms - latency_ms);
    }
    return out;
}
}  // namespace aim
