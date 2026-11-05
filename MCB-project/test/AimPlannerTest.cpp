// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
// Ported from thornbots_pkg/test/test_point_to_cv_target_core.cpp (float tolerances).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include "subsystems/jetson/AimPlanner.hpp"

using namespace aim;

namespace
{
constexpr float V_MUZZLE = 25.0f;
constexpr float PI = PI_F;
const Vec3 SHOOTER{0.0f, 0.0f, 0.4f};
const float TICK_S = 1.0f / 30.0f;
const std::array<float, 2> RADII = {0.30f, 0.24f};
const std::array<float, 2> FLAT = {0.0f, 0.0f};
const std::array<float, 2> STAGGER = {0.045f, -0.045f};

struct ArmorArgs
{
    Vec3 center{3.0f, 0.0f, 0.3f};
    Vec3 vel{};
    float yaw = PI;
    float w = 0.0f;
};

ArmorState armor(const ArmorArgs& a = {})
{
    return {a.center.x, a.center.y, a.center.z, a.vel.x, a.vel.y, a.vel.z, a.yaw, a.w};
}

float dist(const Vec3& a, const Vec3& b) { return norm(a - b); }
float flight(const Vec3& aim) { return norm(aim - SHOOTER) / V_MUZZLE; }

// Zero age; the gimbal lag puts the aim horizon on the fire horizon.
Shot plan(const ArmorState& state, float horizon, bool spinning,
          const std::array<float, 2>& z_offset = FLAT, ShotOptions opt = {})
{
    opt.gimbal_lag_s = horizon - TICK_S / 2.0f;
    opt.firmware_latency_s = horizon;
    opt.min_fire_delay_s = 0.0f;
    return plan_shot(state, RADII, z_offset, 0.0f, SHOOTER, V_MUZZLE, spinning, TICK_S, opt);
}

ShotOptions iters(int n)
{
    ShotOptions o;
    o.iterations = n;
    return o;
}

// |p + v*(tau + t) - s| = v_muzzle * t, smallest positive root.
float analytic_flight_time(const Vec3& p, const Vec3& v, float tau, float v_muzzle, const Vec3& s)
{
    const Vec3 d = p + v * tau - s;
    const float a = v.x * v.x + v.y * v.y + v.z * v.z - v_muzzle * v_muzzle;
    const float b = 2.0f * (d.x * v.x + d.y * v.y + d.z * v.z);
    const float c = d.x * d.x + d.y * d.y + d.z * d.z;
    const float disc = b * b - 4.0f * a * c;
    const float r1 = (-b - std::sqrt(disc)) / (2.0f * a), r2 = (-b + std::sqrt(disc)) / (2.0f * a);
    return std::min(r1 > 0 ? r1 : 1e9f, r2 > 0 ? r2 : 1e9f);
}
}  // namespace

TEST(AimMath, WrapAndModMatchPython)
{
    EXPECT_NEAR(py_mod(-1.0f, 3.0f), 2.0f, 1e-6);
    EXPECT_NEAR(py_mod(1.0f, -3.0f), -2.0f, 1e-6);
    EXPECT_NEAR(wrap_to_pi(3.0f * PI / 2.0f), -PI / 2.0f, 1e-5);
    EXPECT_NEAR(wrap_to_pi(-PI), -PI, 1e-5);
}

TEST(PlanShot, NonSpinningStationaryAimsAtTheFacingPanelNow)
{
    const auto s = plan(armor(), 0.1f, false);
    ASSERT_TRUE(s.delay_s);
    EXPECT_EQ(*s.delay_s, 0.0f);
    EXPECT_LT(dist(s.aim_pos, {2.7f, 0.0f, 0.3f}), 1e-4f);
}

TEST(PlanShot, NonSpinningPicksTheFacingPanelAndItsPair)
{
    ArmorArgs a;
    a.yaw = 0.0f;
    EXPECT_LT(dist(plan(armor(a), 0.1f, false, STAGGER).aim_pos, {2.7f, 0.0f, 0.345f}), 1e-4f);
    a.yaw = PI / 2.0f;
    EXPECT_LT(dist(plan(armor(a), 0.1f, false, STAGGER).aim_pos, {2.76f, 0.0f, 0.255f}), 1e-4f);
}

TEST(ChoosePanel, PicksTheMostFaceOnPanel)
{
    EXPECT_EQ(choose_panel(0.0f, PI, std::nullopt, 0.0f), 2);
    EXPECT_EQ(choose_panel(PI / 2.0f + 0.1f, PI, std::nullopt, 0.0f), 1);
}

TEST(ChoosePanel, HoldsThePanelUntilAnotherIsClearlyBetter)
{
    const float margin = 0.2f, bearing = PI;
    for (float d : {-0.09f, 0.0f, 0.09f}) {
        const float yaw = PI / 4.0f + d;
        EXPECT_EQ(choose_panel(yaw, bearing, wrap_to_pi(yaw + PI / 2.0f), margin), 1) << d;
    }
    const float yaw = PI / 4.0f - 0.11f;
    EXPECT_EQ(choose_panel(yaw, bearing, wrap_to_pi(yaw + PI / 2.0f), margin), 2);
}

TEST(ChoosePanel, FollowsTheHeldPanelByWorldYawNotIndex)
{
    const float held = PI / 2.0f + 0.6f;
    EXPECT_EQ(choose_panel(0.6f, PI, held, 0.5f), 1);
    EXPECT_EQ(choose_panel(0.6f - PI / 2.0f, PI, held, 0.5f), 2);
}

TEST(ChoosePanel, HeldNormalSurvivesThePiWrap)
{
    EXPECT_EQ(choose_panel(-PI + 0.02f, PI, PI - 0.02f, 0.2f), 0);
}

TEST(PlanShot, NonSpinningHoldsAPanelNear45DegreesAcrossTicks)
{
    ShotOptions opt;
    opt.panel_switch_margin_rad = 0.2f;
    ArmorArgs a;
    a.yaw = PI / 4.0f + 0.15f;
    auto s = plan(armor(a), 0.1f, false, STAGGER, opt);
    ASSERT_TRUE(s.panel_yaw);
    const Vec3 first = s.aim_pos;
    for (float d : {-0.05f, 0.05f, -0.08f, 0.08f}) {
        a.yaw = PI / 4.0f + d;
        opt.held_panel_yaw = s.panel_yaw;
        s = plan(armor(a), 0.1f, false, STAGGER, opt);
        EXPECT_NEAR(s.aim_pos.z, first.z, 1e-5) << d;
    }
    a.yaw = PI / 4.0f - 0.05f;
    EXPECT_NE(plan(armor(a), 0.1f, false, STAGGER).aim_pos.z, first.z);
}

TEST(PlanShot, SpinningReportsNoPanelYaw)
{
    ArmorArgs a;
    a.w = 9.0f;
    EXPECT_FALSE(plan(armor(a), 0.1f, true).panel_yaw);
}

TEST(PlanShot, SpinningWithoutLeadIgnoresAPreviousPanelHold)
{
    ArmorArgs a;
    a.yaw = PI / 4.0f - 0.1f;
    ShotOptions opt;
    opt.lead = false;
    opt.held_panel_yaw = PI * 3.0f / 4.0f;
    opt.panel_switch_margin_rad = 0.5f;
    const auto spinning = plan(armor(a), 0.1f, true, STAGGER, opt);
    EXPECT_FALSE(spinning.panel_yaw);
    opt.held_panel_yaw.reset();
    EXPECT_LT(dist(spinning.aim_pos, plan(armor(a), 0.1f, true, STAGGER, opt).aim_pos), 1e-5f);
}

TEST(PlanShot, NonSpinningCrossingPanelMeetsTheInterceptCondition)
{
    const float horizon = 0.12f;
    ArmorArgs a;
    a.vel = {0.0f, 2.0f, 0.0f};
    const auto s = plan(armor(a), horizon, false, FLAT, iters(50));
    const float t = analytic_flight_time({2.7f, 0.0f, 0.3f}, {0.0f, 2.0f, 0.0f}, horizon, V_MUZZLE, SHOOTER);
    EXPECT_NEAR(s.aim_pos.y, 2.0f * (horizon + t), 1e-4);
    EXPECT_NEAR(norm(s.aim_pos - SHOOTER), V_MUZZLE * t, 1e-3);
}

TEST(PlanShot, NonSpinningLeadsTheTangentialPanelVelocity)
{
    ArmorArgs a;
    a.w = 1.0f;
    EXPECT_LT(plan(armor(a), 0.1f, false).aim_pos.y, -0.02f);
}

namespace
{
float alignment_error(const ArmorState& state, float horizon, float delay)
{
    const auto aim = plan(state, horizon, true).aim_pos;
    const float yc = state[1], xc = state[0], yaw = state[6], w = state[7];
    const float t_impact = horizon + flight(aim) + delay;
    const float bearing = std::atan2(SHOOTER.y - yc, SHOOTER.x - xc);
    const float phase = py_mod(yaw + w * t_impact - bearing, PI / 2.0f);
    return std::min(phase, PI / 2.0f - phase);
}
}  // namespace

TEST(PlanShot, SpinningDelayLandsAPanelSquareToTheShooter)
{
    const float w = 2.0f * PI * 1.5f;
    int fired = 0;
    for (int i = 0; i < 40; ++i) {
        const float yaw = PI + i * (PI / 2.0f) / 40.0f;
        for (float spin : {w, -w}) {
            ArmorArgs a;
            a.yaw = yaw;
            a.w = spin;
            const auto state = armor(a);
            const auto s = plan(state, 0.08f, true);
            if (!s.delay_s) continue;
            ++fired;
            EXPECT_GE(*s.delay_s, 0.0f);
            EXPECT_LT(*s.delay_s, TICK_S);
            EXPECT_LT(alignment_error(state, 0.08f, *s.delay_s), 0.02f);
        }
    }
    const float expected = 2 * 40 * (TICK_S * w) / (PI / 2.0f);
    EXPECT_LE(std::abs(fired - expected), 4);
}

TEST(PlanShot, SpinningAimsOnTheCenterToShooterLine)
{
    ArmorArgs a;
    a.center = {3.0f, 1.0f, 0.3f};
    a.w = 9.0f;
    const auto aim = plan(armor(a), 0.1f, true).aim_pos;
    EXPECT_NEAR(std::atan2(aim.y - 1.0f, aim.x - 3.0f), std::atan2(SHOOTER.y - 1.0f, SHOOTER.x - 3.0f), 1e-5);
    const float r = std::hypot(aim.x - 3.0f, aim.y - 1.0f);
    EXPECT_TRUE(std::abs(r - RADII[0]) < 1e-5f || std::abs(r - RADII[1]) < 1e-5f);
}

TEST(PlanShot, SpinningAimsAtTheArrivingPairsRadiusAndHeight)
{
    const float w = 9.0f;
    std::set<int> seen;
    for (int i = 0; i < 160; ++i) {
        ArmorArgs a;
        a.yaw = PI + i * (2.0f * PI) / 160.0f;
        a.w = w;
        const auto state = armor(a);
        const auto s = plan(state, 0.08f, true, STAGGER);
        if (!s.delay_s) continue;
        const float t_impact = 0.08f + flight(s.aim_pos) + *s.delay_s;
        const int k = static_cast<int>(py_mod(std::nearbyint((PI - (state[6] + w * t_impact)) / (PI / 2.0f)), 4.0f));
        EXPECT_NEAR(std::hypot(s.aim_pos.x - 3.0f, s.aim_pos.y), RADII[k % 2], 1e-4);
        EXPECT_NEAR(s.aim_pos.z, 0.3f + STAGGER[k % 2], 1e-5);
        seen.insert(k % 2);
    }
    EXPECT_EQ(seen, (std::set<int>{0, 1}));
}

TEST(PlanShot, AimHorizonSetsTheLeadAndFireHorizonTheTiming)
{
    ArmorArgs a;
    a.vel = {0.0f, 2.0f, 0.0f};
    a.w = 9.0f;
    const auto state = armor(a);
    ShotOptions opt;
    opt.gimbal_lag_s = 0.03f - TICK_S / 2.0f;
    opt.firmware_latency_s = 0.08f;
    const auto near_ = plan_shot(state, {0.27f, 0.27f}, FLAT, 0.0f, SHOOTER, V_MUZZLE, true, TICK_S, opt);
    opt.gimbal_lag_s = 0.08f - TICK_S / 2.0f;
    const auto far_ = plan_shot(state, {0.27f, 0.27f}, FLAT, 0.0f, SHOOTER, V_MUZZLE, true, TICK_S, opt);
    EXPECT_NEAR(far_.aim_pos.y - near_.aim_pos.y, 2.0f * 0.05f, 0.01);
    EXPECT_EQ(near_.delay_s.has_value(), far_.delay_s.has_value());
    if (near_.delay_s && far_.delay_s) {
        EXPECT_EQ(*near_.delay_s, *far_.delay_s);
    }
}

TEST(PlanShot, ChaseAimsAtTheFacingPanelAndFiresOnceSettled)
{
    const float w = 9.0f;
    const float quarter = (PI / 2.0f) / w;
    int fired = 0;
    for (int i = 0; i < 160; ++i) {
        ArmorArgs a;
        a.yaw = PI + i * (2.0f * PI) / 160.0f;
        a.w = w;
        const auto state = armor(a);
        ShotOptions opt;
        opt.gimbal_lag_s = 0.02f - TICK_S / 2.0f;
        opt.firmware_latency_s = 0.05f;
        opt.chase_settle_s = 0.3f * quarter;
        opt.chase_margin_s = 0.1f * quarter;
        const auto s = plan_shot(state, RADII, STAGGER, 0.0f, SHOOTER, V_MUZZLE, true, TICK_S, opt);
        if (!s.delay_s) continue;
        ++fired;
        EXPECT_NEAR(*s.delay_s, py_mod(0.02f - 0.05f, TICK_S), 1e-5);
        const float t_impact = 0.02f + flight(s.aim_pos);
        float best_miss = 1e9f;
        for (int k = 0; k < 4; ++k) {
            const float yaw_k = state[6] + w * t_impact + k * PI / 2.0f;
            best_miss = std::min(best_miss, dist(s.aim_pos, {3.0f + RADII[k % 2] * std::cos(yaw_k), RADII[k % 2] * std::sin(yaw_k), 0.3f + STAGGER[k % 2]}));
        }
        EXPECT_LT(best_miss, 0.01f);
    }
    EXPECT_LE(std::abs(fired - 0.6f * 160), 4);
}

TEST(PlanShot, ExtrapolatesWithAcceleration)
{
    const float horizon = 0.1f;
    ArmorArgs a;
    a.vel = {0.0f, 2.0f, 0.0f};
    ShotOptions opt = iters(50);
    opt.accel = {0.0f, -6.0f, 0.0f};
    const auto s = plan(armor(a), horizon, false, FLAT, opt);
    const float t = horizon + flight(s.aim_pos);
    EXPECT_NEAR(s.aim_pos.y, 2.0f * t - 3.0f * t * t, 1e-4);
    EXPECT_NEAR(s.aim_pos.x, 2.7f, 1e-4);
    EXPECT_GT(plan(armor(a), horizon, false, FLAT, iters(50)).aim_pos.y - s.aim_pos.y, 0.05f);
}

TEST(PlanShot, WithoutLeadAimsAtTheCurrentEstimateAndFiresNow)
{
    ArmorArgs a;
    a.vel = {0.0f, 4.0f, 0.0f};
    a.w = 9.0f;
    ShotOptions opt;
    opt.lead = false;
    const auto s = plan(armor(a), 0.3f, true, FLAT, opt);
    ASSERT_TRUE(s.delay_s);
    EXPECT_EQ(*s.delay_s, 0.0f);
    EXPECT_LT(std::abs(s.aim_pos.y), 1e-5f);
}

namespace
{
Vec3 exit_point(float horizon, const Vec3& sv) { return SHOOTER + sv * horizon; }
Vec3 fly(const Vec3& gun, float horizon, const Vec3& sv, float t)
{
    const Vec3 start = exit_point(horizon, sv);
    const Vec3 d = gun - start;
    return start + (sv + d * (V_MUZZLE / norm(d))) * t;
}
}  // namespace

TEST(PlanShot, MovingShooterLandsOnThePanel)
{
    const std::vector<Vec3> shooter_vels = {{0, 1, 0}, {0, -2, 0}, {1.5f, 0.5f, 0}};
    const std::vector<Vec3> target_vels = {{0, 0, 0}, {0, 2, 0}};
    for (const auto& sv : shooter_vels) {
        for (const auto& tv : target_vels) {
            const float horizon = 0.1f;
            ArmorArgs a;
            a.vel = tv;
            const auto state = armor(a);
            ShotOptions opt = iters(50);
            opt.shooter_vel = sv;
            const Vec3 gun = plan(state, horizon, false, FLAT, opt).aim_pos;
            const Vec3 still = plan(state, horizon, false, FLAT, iters(50)).aim_pos;
            const float t = dist(gun, exit_point(horizon, sv)) / V_MUZZLE;
            const Vec3 panel{2.7f + tv.x * (horizon + t), tv.y * (horizon + t), 0.3f};
            EXPECT_LT(dist(fly(gun, horizon, sv, t), panel), 1e-3f);
            const float t_still = dist(still, exit_point(horizon, sv)) / V_MUZZLE;
            EXPECT_GT(dist(fly(still, horizon, sv, t_still), panel), 0.1f);
        }
    }
}

TEST(PlanShot, MovingShooterChaseLandsOnAFacingPanel)
{
    const float w = 9.0f;
    const Vec3 sv{0.0f, 1.0f, 0.0f};
    ArmorArgs a;
    a.yaw = PI + 0.3f;
    a.w = w;
    a.vel = {0.0f, 1.0f, 0.0f};
    const auto state = armor(a);
    ShotOptions opt = iters(50);
    opt.gimbal_lag_s = 0.02f - TICK_S / 2.0f;
    opt.firmware_latency_s = 0.05f;
    opt.shooter_vel = sv;
    opt.chase_settle_s = 0.0f;
    const auto s = plan_shot(state, RADII, FLAT, 0.0f, SHOOTER, V_MUZZLE, true, TICK_S, opt);
    ASSERT_TRUE(s.delay_s);
    const float t = dist(s.aim_pos, exit_point(0.02f, sv)) / V_MUZZLE;
    const float t_impact = 0.02f + t;
    const Vec3 center{3.0f, t_impact, 0.3f};
    const Vec3 shot = fly(s.aim_pos, 0.02f, sv, t);
    float miss = 1e9f;
    for (int k = 0; k < 4; ++k) {
        const float yaw_k = state[6] + w * t_impact + k * PI / 2.0f;
        miss = std::min(miss, dist(shot, {center.x + RADII[k % 2] * std::cos(yaw_k), center.y + RADII[k % 2] * std::sin(yaw_k), 0.3f}));
    }
    EXPECT_LT(miss, 2e-3f);
}

TEST(FireTiming, ExitNeverSendsUnderTheFloor)
{
    EXPECT_FLOAT_EQ(fire_exit_s(0.020f, 0.040f), 0.040f);
    EXPECT_FLOAT_EQ(fire_exit_s(0.094f, 0.040f), 0.094f);
    EXPECT_EQ(wire_fire_delay_ms(0.0f, fire_exit_s(0.020f, 0.040f)), 40);
    EXPECT_EQ(wire_fire_delay_ms(-0.001f, 0.040f), 40);
}

TEST(FireTiming, WireDelayPreservesAlignmentAfterFirmwareLatencySubtraction)
{
    EXPECT_EQ(wire_fire_delay_ms(0.0f, 0.094f), 94);
    EXPECT_EQ(wire_fire_delay_ms(0.02f, 0.094f) - 94, 20);
    EXPECT_EQ(wire_fire_delay_ms(0.05f, 0.094f) - 94, 50);
    EXPECT_EQ(wire_fire_delay_ms(70.0f, 0.08f), 65535);
}

TEST(PlanShot, OneHzAlignmentGatingHoldsYawAndMeetsTheExitHorizon)
{
    for (float w : {2 * PI, -2 * PI}) {
        int fired = 0;
        for (int i = 0; i < 160; ++i) {
            ArmorArgs a;
            a.center.x = 1.3f;
            a.yaw = PI + i * 2 * PI / 160;
            a.w = w;
            ShotOptions opt;
            opt.gimbal_lag_s = 0;
            const float age = .013f, tick = .025f;
            const auto s = plan_shot(armor(a), {.24f, .24f}, FLAT, age, SHOOTER, V_MUZZLE, true, tick, opt);
            EXPECT_NEAR(s.aim_pos.y, 0, 1e-4);
            if (!s.delay_s) continue;
            ++fired;
            EXPECT_GE(*s.delay_s, 0);
            EXPECT_LT(*s.delay_s, tick);
            const float impact = age + fire_exit_s(opt.firmware_latency_s, opt.min_fire_delay_s) + *s.delay_s + flight(s.aim_pos);
            const float phase = py_mod(a.yaw + w * impact - PI, PI / 2);
            EXPECT_LT(std::min(phase, PI / 2 - phase), 1e-3f);
        }
        EXPECT_GT(fired, 0);
    }
}

TEST(PlanShot, SpinFirePolicyDoesNotChangeNonspinningMovingPrediction)
{
    for (const Vec3& velocity : {Vec3{2, 0, 0}, Vec3{0, 2, 0}, Vec3{1, 1, 0}}) {
        ArmorArgs a;
        a.vel = velocity;
        ShotOptions gated, chase;
        gated.gimbal_lag_s = chase.gimbal_lag_s = 0;
        chase.chase_settle_s = .025f;
        chase.chase_margin_s = .025f;
        const auto g = plan_shot(armor(a), RADII, STAGGER, .02f, SHOOTER, V_MUZZLE, false, .025f, gated);
        const auto c = plan_shot(armor(a), RADII, STAGGER, .02f, SHOOTER, V_MUZZLE, false, .025f, chase);
        EXPECT_LT(dist(g.aim_pos, c.aim_pos), 1e-6f);
        EXPECT_EQ(g.delay_s, c.delay_s);
        EXPECT_EQ(g.panel_yaw, c.panel_yaw);
    }
}

// ---- FireLock (millisecond clock) ----

TEST(FireLock, MovingTargetWaitsOneSecondAndIdentityChangesRestart)
{
    FireLock lock;
    EXPECT_FALSE(lock.update(2000, 7, true, false, 0.2f, 1000, 0.1f, 300));
    EXPECT_FALSE(lock.update(2999, 7, true, false, 0.2f, 1000, 0.1f, 300));
    EXPECT_TRUE(lock.update(3000, 7, true, false, 0.2f, 1000, 0.1f, 300));
    EXPECT_FALSE(lock.update(3100, 8, true, false, 0.2f, 1000, 0.1f, 300));
    EXPECT_TRUE(lock.update(4100, 8, true, false, 0.2f, 1000, 0.1f, 300));
}

TEST(FireLock, LongIneligibleGapAndClockRewindRestart)
{
    FireLock lock;
    EXPECT_FALSE(lock.update(1000, 1, true, true, 0, 1000, 0.1f, 300));
    EXPECT_TRUE(lock.update(2000, 1, true, true, 0, 1000, 0.1f, 300));
    EXPECT_FALSE(lock.update(2100, 1, false, true, 0, 1000, 0.1f, 300));
    EXPECT_FALSE(lock.update(2500, 1, true, true, 0, 1000, 0.1f, 300));
    EXPECT_TRUE(lock.update(3500, 1, true, true, 0, 1000, 0.1f, 300));
    EXPECT_FALSE(lock.update(3000, 1, true, true, 0, 1000, 0.1f, 300));
}

TEST(FireLock, SurvivesMillisecondClockWrap)
{
    FireLock lock;
    const uint32_t t0 = 0xFFFFFF00U;
    EXPECT_FALSE(lock.update(t0, 1, true, true, 0, 1000, 0.1f, 300));
    EXPECT_FALSE(lock.update(t0 + 999, 1, true, true, 0, 1000, 0.1f, 300));
    EXPECT_TRUE(lock.update(t0 + 1000, 1, true, true, 0, 1000, 0.1f, 300));
}

TEST(FireLock, StationaryBypassRequiresLowSpeedAndNoSpin)
{
    FireLock lock;
    EXPECT_TRUE(lock.update(1000, 1, true, false, 0.05f, 1000, 0.1f, 300));
    EXPECT_FALSE(lock.update(1100, 1, true, true, 0, 1000, 0.1f, 300));
    EXPECT_FALSE(lock.update(1200, 1, true, false, 0.1f, 1000, 0.1f, 300));
    EXPECT_FALSE(lock.update(1300, 1, false, false, 0, 1000, 0.1f, 300));
}

TEST(FireLock, ZeroDisablesOnlyTheDwell)
{
    FireLock lock;
    EXPECT_TRUE(lock.update(1000, 1, true, true, 0, 0, 0.0f, 300));
    EXPECT_FALSE(lock.update(1100, 1, false, true, 0, 0, 0.0f, 300));
}

TEST(FireLock, BriefGapCountsTowardDwellButNeverAllowsFire)
{
    FireLock lock;
    EXPECT_FALSE(lock.update(1000, 7, true, true, 0, 1000, 0.1f, 300));
    for (int frame = 0; frame < 5; ++frame)
        EXPECT_FALSE(lock.update(1500 + frame * 33, 7, false, true, 0, 1000, 0.1f, 300));
    EXPECT_FALSE(lock.update(1700, 7, true, true, 0, 1000, 0.1f, 300));
    EXPECT_TRUE(lock.update(2000, 7, true, true, 0, 1000, 0.1f, 300));
    EXPECT_FALSE(lock.update(2100, 7, false, true, 0, 1000, 0.1f, 300));
    EXPECT_FALSE(lock.update(2200, 8, false, true, 0, 1000, 0.1f, 300));
    EXPECT_FALSE(lock.update(2300, 7, true, true, 0, 1000, 0.1f, 300));
}

// ---- AimPlanner fire gating ----

namespace
{
TargetInput target(uint32_t id = 5)
{
    TargetInput in;
    in.center = {3.0f, 0.0f, 0.3f};
    in.velocity = {0.5f, 0.0f, 0.0f};  // moving: needs the dwell
    in.yaw = PI;
    in.radius = RADII;
    in.z_offset = FLAT;
    in.panel = {2.7f, 0.0f, 0.3f};
    in.state_age_s = 0.02f;
    in.detection_age_s = 0.03f;
    in.track_id = id;
    in.confidence = 200.0f / 255.0f;
    in.valid = true;
    return in;
}
}  // namespace

TEST(AimPlannerFire, MovingTargetFiresOnlyAfterTheDwellWithTheJetsonDelay)
{
    AimPlanner p;
    auto in = target();
    p.observe(0, in);
    EXPECT_FALSE(p.plan(0, in, SHOOTER, {}).fire);
    EXPECT_FALSE(p.plan(999, in, SHOOTER, {}).fire);
    const Plan plan1 = p.plan(1000, in, SHOOTER, {});
    ASSERT_TRUE(plan1.fire);
    EXPECT_EQ(plan1.request_delay_ms, 20);  // 40 ms exit floor less the 20 ms indexer latency
}

TEST(AimPlannerFire, StationaryTargetBypassesTheDwellButSpinDoesNot)
{
    AimPlanner p;
    auto in = target();
    in.velocity = {0.03f, 0.04f, 0.0f};
    p.observe(0, in);
    EXPECT_TRUE(p.plan(0, in, SHOOTER, {}).fire);
    in.yaw_rate = 9.0f;
    AimPlanner q;
    q.observe(0, in);
    EXPECT_FALSE(q.plan(0, in, SHOOTER, {}).fire);
}

TEST(AimPlannerFire, ConfidenceAndDetectionAgeGateFire)
{
    AimPlanner p;
    auto in = target();
    in.velocity = {};
    in.confidence = 127.0f / 255.0f;
    EXPECT_FALSE(p.plan(0, in, SHOOTER, {}).fire);
    in.confidence = 128.0f / 255.0f;
    EXPECT_TRUE(p.plan(10, in, SHOOTER, {}).fire);
    in.detection_age_s = 0.151f;
    EXPECT_FALSE(p.plan(20, in, SHOOTER, {}).fire);
    in.detection_age_s = 0.150f;
    EXPECT_TRUE(p.plan(30, in, SHOOTER, {}).fire);
}

TEST(AimPlannerFire, InvalidStateAimsAtThePanelWithoutLeadOrFire)
{
    AimPlanner p;
    auto in = target();
    in.velocity = {};
    in.valid = false;
    in.panel = {1.0f, 2.0f, 3.0f};
    const Plan out = p.plan(0, in, SHOOTER, {});
    EXPECT_FALSE(out.fire);
    EXPECT_EQ(out.aim.x, 1.0f);
    EXPECT_EQ(out.aim.y, 2.0f);
    EXPECT_EQ(out.aim.z, 3.0f);
}

TEST(AimPlannerFire, TrackChangeAndResetRestartTheDwell)
{
    AimPlanner p;
    auto in = target(5);
    p.observe(0, in);
    p.plan(0, in, SHOOTER, {});
    ASSERT_TRUE(p.plan(1000, in, SHOOTER, {}).fire);
    in = target(6);
    p.observe(1010, in);
    EXPECT_FALSE(p.plan(1010, in, SHOOTER, {}).fire);
    EXPECT_TRUE(p.plan(2010, in, SHOOTER, {}).fire);
    p.reset();
    EXPECT_FALSE(p.plan(2020, in, SHOOTER, {}).fire);
}

TEST(AimPlannerFire, SpinHysteresisEntersAbove3AndExitsBelow2)
{
    AimPlanner p;
    auto in = target();
    in.velocity = {};
    in.yaw_rate = 2.5f;
    EXPECT_FALSE(p.plan(0, in, SHOOTER, {}).spinning);
    in.yaw_rate = 3.1f;
    EXPECT_TRUE(p.plan(25, in, SHOOTER, {}).spinning);
    in.yaw_rate = 2.5f;
    EXPECT_TRUE(p.plan(50, in, SHOOTER, {}).spinning);
    in.yaw_rate = 1.9f;
    EXPECT_FALSE(p.plan(75, in, SHOOTER, {}).spinning);
}
