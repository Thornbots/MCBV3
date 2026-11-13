#include "AimPlanner.hpp"

#include <algorithm>
#include <cmath>

namespace aim {
float norm(const Vec3& a) { return std::sqrt(dot(a, a)); }
float py_mod(float a, float b) { return a - b * std::floor(a / b); }
float wrap_to_pi(float a) { return py_mod(a + PI_F, 2 * PI_F) - PI_F; }
namespace {
int quarter_index(float turns) { return static_cast<int>(py_mod(std::nearbyint(turns), 4)); }
bool finite(const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
Vec3 center_at(const TargetInput& in, float time) { return in.center + in.velocity * time + in.acceleration * (0.5f * time * time); }
float bearing_at(const TargetInput& in, float time, const Vec3& shooter) {
    const Vec3 c = center_at(in, time);
    return std::atan2(shooter.y - c.y, shooter.x - c.x);
}
}  // namespace

int choose_panel(float yaw, float bearing, std::optional<float> held_yaw, float margin_rad) {
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

bool usable(const TargetInput& in) {
    if (!in.valid || !finite(in.center) || !finite(in.velocity) || !finite(in.acceleration) || !finite(in.panel) || !std::isfinite(in.yaw) || !std::isfinite(in.yaw_rate) ||
        !std::isfinite(in.state_age_s) || in.state_age_s < 0)
        return false;
    for (int i = 0; i < 2; ++i)
        if (!std::isfinite(in.radius[i]) || in.radius[i] < 0 || !std::isfinite(in.z_offset[i])) return false;
    return true;
}

Panel panel_at(const TargetInput& in, int k, float time) {
    const float yaw = in.yaw + in.yaw_rate * time + k * QUARTER_TURN;
    const float c = std::cos(yaw), s = std::sin(yaw), r = in.radius[k % 2];
    constexpr float ct = 0.9659258263f, st = 0.2588190451f;
    return {
        center_at(in, time) + Vec3{r * c, r * s, in.z_offset[k % 2]},
        in.velocity + in.acceleration * time + Vec3{-r * s * in.yaw_rate, r * c * in.yaw_rate, 0},
        {ct * c, ct * s, st},
        {-s, c, 0},
        {-st * c, -st * s, ct},
        {-ct * s * in.yaw_rate, ct * c * in.yaw_rate, 0}};
}

void AimPlanner::reset() {
    spinning_ = false;
    has_track_ = false;
    held_panel_yaw_.reset();
    pitch_seed_.reset();
    time_seed_.reset();
}
void AimPlanner::observe(uint32_t, const TargetInput& in) {
    if (in.clear || !usable(in)) {
        reset();
        return;
    }
    if (!has_track_ || track_id_ != in.track_id) reset();
    has_track_ = true;
    track_id_ = in.track_id;
}

Plan AimPlanner::plan(uint32_t now_ms, const TargetInput& in, const Vec3& pivot_now, const Vec3& shooter_vel) {
    observe(now_ms, in);
    if (!has_track_ || !finite(pivot_now) || !finite(shooter_vel)) return {};
    spinning_ = std::abs(in.yaw_rate) > (spinning_ ? t.spin_exit_rad_s : t.spin_enter_rad_s);
    const bool gating = std::abs(in.yaw_rate) > t.shot_gating_rad_s ||
                        (!t.chase && std::abs(in.yaw_rate) > 1e-6f);
    BallisticModel m = t.ballistics;
    m.speed = t.v_muzzle_mps;
    const float horizon = in.state_age_s + t.firmware_latency_s;
    const Vec3 exit = pivot_now + shooter_vel * t.firmware_latency_s;
    auto select = [&](float flight) {
        const float h = horizon + flight;
        return choose_panel(in.yaw + in.yaw_rate * h, bearing_at(in, h, exit + shooter_vel * flight), spinning_ ? std::nullopt : held_panel_yaw_, spinning_ ? 0 : t.panel_switch_margin_rad);
    };
    const float initial = time_seed_.value_or(norm(center_at(in, horizon) - exit) / m.speed);
    const int first = select(initial);
    auto solve = [&](int k) -> Plan {
        float flight = initial;
        std::optional<float> seed = pitch_seed_;
        Plan out;
        for (unsigned i = 0; i <= 3; ++i) {
            Panel panel = panel_at(in, k, horizon + flight);
            if (gating) {
                // Hold the face-crossing line; translation still gets ballistic lead.
                const float h = horizon + flight;
                const Vec3 c = center_at(in, h), cv = in.velocity + in.acceleration * h;
                const Vec3 toward = exit + shooter_vel * flight - c;
                const float range = std::hypot(toward.x, toward.y);
                if (!(range > 1e-6f)) return {};
                const Vec3 n{toward.x / range, toward.y / range, 0};
                const Vec3 relative = shooter_vel - cv;
                const Vec3 ndot = (Vec3{relative.x, relative.y, 0} - n * dot(n, relative)) * (1 / range);
                panel.center = c + n * in.radius[k % 2] + Vec3{0, 0, in.z_offset[k % 2]};
                panel.velocity = cv + ndot * in.radius[k % 2];
            }
            const Vec3 hit = panel.center - shooter_vel * flight;
            const Vec3 d = hit - exit;
            const float range = std::hypot(d.x, d.y);
            const float forward2 = range * range - m.bore_right * m.bore_right;
            if (!(forward2 > 0)) return {};
            const float forward = std::sqrt(forward2);
            const auto b = solve_ballistic(forward, d.z, m, seed);
            if (!b) return {};
            seed = b->pitch;
            const Vec3 rel = panel.velocity - shooter_vel;
            const float rt = (d.x * rel.x + d.y * rel.y) / forward;
            const float tt = b->time_range * rt + b->time_height * rel.z;
            const float denom = 1 - tt;
            if (!std::isfinite(denom) || std::abs(denom) < 0.1f) return {};
            const float residual = b->time - flight;
            if (std::abs(residual) <= 0.0001f) {
                const float time_rate = tt / denom;
                const Vec3 delta_rate = rel * (1 + time_rate);
                const float range_rate = (d.x * delta_rate.x + d.y * delta_rate.y) / range;
                const float forward_rate = range * range_rate / forward;
                out.aim = hit;
                out.aim_rate = panel.velocity * (1 + time_rate) - shooter_vel * time_rate;
                out.yaw = std::atan2(d.y, d.x) + std::asin(m.bore_right / range);
                out.pitch = b->pitch;
                out.yaw_rate = (d.x * delta_rate.y - d.y * delta_rate.x) / (range * range) - m.bore_right * range_rate / (range * forward);
                out.pitch_rate = b->pitch_range * forward_rate + b->pitch_height * delta_rate.z;
                out.flight_time = flight;
                out.time_rate = time_rate;
                out.panel = k;
                out.valid = std::isfinite(out.yaw_rate) && std::isfinite(out.pitch_rate);
                out.spinning = spinning_;
                return out;
            }
            if (i == 3) return {};
            flight += residual / denom;
            if (!(flight > 0 && flight < 2)) return {};
        }
        return {};
    };
    Plan out = solve(first);
    if (!out.valid) return {};
    // A fixed-panel solve stays smooth; solve the neighbor separately at a chase boundary.
    const int next = select(out.flight_time);
    if (next != first) {
        out = solve(next);
        if (!out.valid || select(out.flight_time) != next) return {};
    }
    held_panel_yaw_ = wrap_to_pi(in.yaw + in.yaw_rate * (horizon + out.flight_time) + out.panel * QUARTER_TURN);
    pitch_seed_ = out.pitch;
    time_seed_ = out.flight_time;
    return out;
}

bool shot_window(const TargetInput& in, int panel, float flight_time, const Vec3& pivot_now, const Vec3& shooter_vel, bool spinning, const AimTuning& t) {
    const bool gating = std::abs(in.yaw_rate) > t.shot_gating_rad_s ||
                        (!t.chase && std::abs(in.yaw_rate) > 1e-6f);
    if (!spinning && !gating) return true;
    const float h = in.state_age_s + t.firmware_latency_s + flight_time;
    const Vec3 shooter = pivot_now + shooter_vel * (t.firmware_latency_s + flight_time);
    const float bearing = bearing_at(in, h, shooter);
    const float theta = wrap_to_pi(in.yaw + in.yaw_rate * h + panel * QUARTER_TURN - bearing);
    if (std::abs(theta) > QUARTER_TURN / 2) return false;
    const float w = std::abs(in.yaw_rate);
    if (w < 1e-6f) return false;
    const float phase = py_mod(theta + QUARTER_TURN / 2, QUARTER_TURN);
    const float since = (in.yaw_rate > 0 ? phase : QUARTER_TURN - phase) / w;
    const float until = QUARTER_TURN / w - since;
    if (!gating) return since >= t.chase_settle_s && until >= t.chase_margin_s;
    // Only the imminent face-on crossing; the measured outline gate also applies.
    return std::abs(theta) <= w * 0.001f;
}

std::optional<Impact> predict_impact(const TargetInput& in, const Plan& plan, const Vec3& pivot_now, const Vec3& shooter_vel, const GimbalMotion& measured, const AimTuning& t) {
    if (!plan.valid || !usable(in)) return std::nullopt;
    const auto& m = t.ballistics;
    const float latency = t.firmware_latency_s;
    const float yaw = measured.yaw + measured.yaw_rate * latency;
    const float pitch = measured.pitch + measured.pitch_rate * latency;
    const Vec3 forward{std::cos(yaw), std::sin(yaw), 0};
    const Vec3 right{forward.y, -forward.x, 0};
    const float c = std::cos(pitch), s = std::sin(pitch);
    const Vec3 origin = pivot_now + shooter_vel * latency + right * m.bore_right + forward * (m.bore_forward * c + m.bore_up * s) + Vec3{0, 0, -m.bore_forward * s + m.bore_up * c};
    const float vh = t.v_muzzle_mps * std::cos(pitch + m.bias);
    const float vz = -t.v_muzzle_mps * std::sin(pitch + m.bias);
    if (!(vh > 0 && m.drag >= 0)) return std::nullopt;
    float time = plan.flight_time;
    for (unsigned i = 0; i <= 3; ++i) {
        if (!(time > 0 && time < 2)) return std::nullopt;
        const float q = m.drag * vh * time;
        const float distance = m.drag > 0 ? std::log1p(q) / m.drag : vh * time;
        const Vec3 p = origin + shooter_vel * time + forward * distance + Vec3{0, 0, vz * time - 0.5f * m.gravity * time * time};
        const Vec3 v = shooter_vel + forward * (vh / (1 + q)) + Vec3{0, 0, vz - m.gravity * time};
        const Panel panel = panel_at(in, plan.panel, in.state_age_s + latency + time);
        const Vec3 delta = p - panel.center;
        const float residual = dot(delta, panel.normal);
        const float deriv = dot(v - panel.velocity, panel.normal) + dot(delta, panel.normal_rate);
        // A front-side forward crossing requires motion into the outward normal.
        if (!std::isfinite(residual) || !std::isfinite(deriv) || deriv >= -1e-4f) return std::nullopt;
        if (std::abs(residual) <= 1e-6f) {
            const float u = dot(delta, panel.width), h = dot(delta, panel.height);
            return Impact{time, u, h, inside_panel(u, h)};
        }
        if (i == 3) break;
        time -= residual / deriv;
    }
    return std::nullopt;
}

void AimControl::reset() {
    state_ = State::WAITING;
    solved_ = {};
    dirty_ = false;
    replanned_ = false;
    planner_.reset();
}
bool AimControl::observe(uint32_t receipt_ms, const TargetInput& in) {
    if (in.clear) {
        reset();
        return true;
    }
    if (!usable(in)) return false;
    // Expiry is evaluated before a refresh, so a long gap starts a fresh assignment.
    if (state_ == State::TRACKING && receipt_ms - received_ms_ >= 600) reset();
    planner_.observe(receipt_ms, in);
    if (state_ == State::WAITING || in.track_id != held_.track_id) solved_ = {};
    held_ = in;
    received_ms_ = receipt_ms;
    state_ = State::TRACKING;
    dirty_ = true;
    return true;
}
TargetInput AimControl::aged(uint32_t now_ms) const {
    auto in = held_;
    in.state_age_s += (now_ms - received_ms_) * 0.001f;
    in.detection_age_s += (now_ms - received_ms_) * 0.001f;
    return in;
}
Plan AimControl::tick(uint32_t now_ms, const Vec3& pivot_now, const Vec3& shooter_vel) {
    replanned_ = false;
    if (state_ == State::TRACKING && now_ms - received_ms_ >= 600) reset();
    if (state_ == State::WAITING) return {};
    if (dirty_ || now_ms - solved_ms_ >= static_cast<uint32_t>(tuning().replan_period_s * 1000 + 0.5f)) {
        solved_ = planner_.plan(now_ms, aged(now_ms), pivot_now, shooter_vel);
        solved_ms_ = now_ms;
        dirty_ = false;
        replanned_ = true;
    }
    auto out = solved_;
    // The smooth solution describes muzzle exit. Back-project to the current
    // controller tick so physical rate extrapolation reaches that solution at exit.
    const float dt = (now_ms - solved_ms_) * 0.001f - tuning().firmware_latency_s;
    out.aim = out.aim + out.aim_rate * dt;
    out.yaw += out.yaw_rate * dt;
    out.pitch += out.pitch_rate * dt;
    out.flight_time += out.time_rate * (now_ms - solved_ms_) * 0.001f;
    return out;
}
bool AimControl::can_fire(uint32_t now_ms, const Vec3& pivot_now, const Vec3& shooter_vel, const GimbalMotion& measured) const {
    if (state_ != State::TRACKING || now_ms - received_ms_ >= 600 || dirty_) return false;
    const auto in = aged(now_ms);
    const auto impact = predict_impact(in, solved_, pivot_now, shooter_vel, measured, tuning());
    return impact && impact->inside && shot_window(in, solved_.panel, impact->time, pivot_now, shooter_vel, solved_.spinning, tuning());
}
}  // namespace aim
