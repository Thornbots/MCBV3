// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "subsystems/jetson/AimPlanner.hpp"
using namespace aim;
namespace {
const Vec3 PIVOT{0, 0, 0.4035f};
TargetInput target() {
    TargetInput in;
    in.center = {3, 0, 0.3f};
    in.yaw = PI_F;
    in.radius = {0.25f, 0.25f};
    in.valid = true;
    in.track_id = 1;
    return in;
}
GimbalMotion aligned(const Plan& p) { return {p.yaw, p.pitch, 0, 0}; }
float old_pitch(float range, float z, const BallisticModel& m) {
    float p = 0;
    for (int i = 0; i < 10; ++i) {
        const auto h = ballistic_height(range, p, m);
        p += (h && h->height > z ? 1 : -1) * PI_F / (4 << i);
    }
    return p;
}
}  // namespace
TEST(Ballistics, MatchesTenStepReticleAndConvergedReference) {
    unsigned cases = 0;
    for (float speed : {22.f, 25.f, 16.f})
        for (float drag : {0.f, .02f})
            for (float forward : {0.f, .173f})
                for (float up : {0.f, .035f})
                    for (float bias : {0.f, .042f})
                        for (float r = 1; r <= 8; r += .25f)
                            for (float z : {-.5f, -.2235f, 0.f, .3f, .8f}) {
                                BallisticModel m{speed, drag, bias, forward, up, 0, 9.80665f};
                                const auto b = solve_ballistic(r, z, m);
                                ASSERT_TRUE(b) << r << " " << z << " " << speed;
                                const float old = old_pitch(r, z, m);
                                const auto h = ballistic_height(r, b->pitch, m);
                                ASSERT_TRUE(h);
                                EXPECT_NEAR(b->pitch, old, .002f);
                                EXPECT_NEAR(b->time, ballistic_height(r, old, m)->time, .0005f);
                                EXPECT_NEAR(h->height, z, .000501f);
                                double lo = -1.2, hi = 1.2;
                                for (int i = 0; i < 40; ++i) {
                                    const double p = (lo + hi) / 2;
                                    const double f = forward * std::cos(p) + up * std::sin(p);
                                    const double u = -forward * std::sin(p) + up * std::cos(p);
                                    const double vh = speed * std::cos(p + bias), vz = -speed * std::sin(p + bias);
                                    const double t = drag > 0 ? std::expm1(drag * (r - f)) / (drag * vh) : (r - f) / vh;
                                    if (u + vz * t - .5 * m.gravity * t * t > z)
                                        lo = p;
                                    else
                                        hi = p;
                                }
                                EXPECT_NEAR(h->height, z, .000501f);
                                EXPECT_NEAR(b->pitch, (lo + hi) / 2, .0006f);
                                EXPECT_LE(b->corrections, 3U);
                                ++cases;
                            }
    EXPECT_EQ(cases, 6960U);
}
TEST(Ballistics, AnalyticDerivativesMatchIndependentDifferences) {
    BallisticModel m;
    const auto b = solve_ballistic(5, -.2f, m);
    ASSERT_TRUE(b);
    const float eps = .002f;
    auto r = solve_ballistic(5 + eps, -.2f, m), z = solve_ballistic(5, -.2f + eps, m);
    ASSERT_TRUE(r && z);
    EXPECT_NEAR((r->pitch - b->pitch) / eps, b->pitch_range, .001);
    EXPECT_NEAR((z->pitch - b->pitch) / eps, b->pitch_height, .001);
    EXPECT_NEAR((r->time - b->time) / eps, b->time_range, .0001);
    EXPECT_NEAR((z->time - b->time) / eps, b->time_height, .0001);
}
TEST(Ballistics, FailsClosedOnUnreachableNonfiniteAndSingularInputs) {
    BallisticModel m;
    EXPECT_FALSE(solve_ballistic(1000, 1000, m));
    EXPECT_FALSE(solve_ballistic(0, 0, m));
    EXPECT_FALSE(solve_ballistic(3, std::numeric_limits<float>::quiet_NaN(), m));
    EXPECT_FALSE(solve_ballistic(3, 0, m, PI_F / 2));
    EXPECT_FALSE(solve_ballistic(3, 0, m, 1.5f));
    m.speed = 0;
    EXPECT_FALSE(solve_ballistic(3, 0, m));
}
TEST(ManagedAim, StartsWaitingAndIdleCannotAssign) {
    AimControl c;
    EXPECT_EQ(c.state(), AimControl::State::WAITING);
    EXPECT_FALSE(c.tick(0, PIVOT, {}).valid);
    EXPECT_FALSE(c.can_fire(0, PIVOT, {}, {}));
    auto in = target();
    in.valid = false;
    EXPECT_FALSE(c.observe(1, in));
    EXPECT_EQ(c.state(), AimControl::State::WAITING);
}
TEST(ManagedAim, AssignsImmediatelyWithoutConfidenceAgeOrDwellGates) {
    AimControl c;
    auto in = target();
    in.confidence = 0;
    in.detection_age_s = 65;
    in.state_age_s = 1;
    ASSERT_TRUE(c.observe(0, in));
    EXPECT_EQ(c.state(), AimControl::State::TRACKING);
    const auto p = c.tick(0, PIVOT, {});
    ASSERT_TRUE(p.valid);
    EXPECT_TRUE(c.can_fire(0, PIVOT, {}, aligned(p)));
}
TEST(ManagedAim, ClearWinsOverValidTrackAndBadGeometry) {
    AimControl c;
    c.observe(0, target());
    const auto p = c.tick(0, PIVOT, {});
    auto clear = target();
    clear.clear = true;
    clear.track_id = 999;
    clear.center.x = std::numeric_limits<float>::quiet_NaN();
    EXPECT_TRUE(c.observe(1, clear));
    EXPECT_EQ(c.state(), AimControl::State::WAITING);
    EXPECT_FALSE(c.can_fire(1, PIVOT, {}, aligned(p)));
    EXPECT_FALSE(c.tick(1, PIVOT, {}).valid);
}
TEST(ManagedAim, ExpiryUsesOnlyReceiptAndIsExactAcrossClockRollover) {
    for (uint32_t start : {0U, UINT32_MAX - 200}) {
        AimControl c;
        auto in = target();
        in.state_age_s = 2;
        c.observe(start, in);
        EXPECT_TRUE(c.tick(start + 599, PIVOT, {}).valid);
        EXPECT_FALSE(c.tick(start + 600, PIVOT, {}).valid);
        EXPECT_EQ(c.state(), AimControl::State::WAITING);
    }
}
TEST(ManagedAim, InvalidAndMalformedFramesCannotExtendLink) {
    for (int malformed = 0; malformed < 3; ++malformed) {
        AimControl c;
        c.observe(0, target());
        c.tick(0, PIVOT, {});
        auto in = target();
        if (malformed == 0) in.valid = false;
        if (malformed == 1) in.yaw = std::numeric_limits<float>::quiet_NaN();
        if (malformed == 2) in.radius[0] = -1;
        EXPECT_FALSE(c.observe(599, in));
        EXPECT_FALSE(c.tick(600, PIVOT, {}).valid);
    }
}
TEST(ManagedAim, RefreshPreservesSpinHysteresisButTrackChangeResetsIt) {
    AimControl c;
    auto in = target();
    in.yaw_rate = 3.5f;
    c.observe(0, in);
    EXPECT_TRUE(c.tick(0, PIVOT, {}).spinning);
    in.yaw_rate = 2.5f;
    c.observe(1, in);
    EXPECT_TRUE(c.tick(1, PIVOT, {}).spinning);
    in.track_id = 2;
    c.observe(2, in);
    EXPECT_FALSE(c.can_fire(2, PIVOT, {}, {}));
    EXPECT_FALSE(c.tick(2, PIVOT, {}).spinning);
    EXPECT_TRUE(c.tick(601, PIVOT, {}).valid);
    EXPECT_FALSE(c.tick(602, PIVOT, {}).valid);
}
TEST(ManagedAim, ReplansAtFiveMsAndAdvancesWithRatesBetweenSolves) {
    AimControl c;
    auto in = target();
    in.velocity = {0, 2, 0};
    c.observe(100, in);
    const auto a = c.tick(100, PIVOT, {});
    ASSERT_TRUE(a.valid);
    EXPECT_TRUE(c.replanned());
    for (uint32_t dt = 1; dt < 5; ++dt) {
        auto b = c.tick(100 + dt, PIVOT, {});
        EXPECT_FALSE(c.replanned());
        EXPECT_NEAR(b.yaw, a.yaw + a.yaw_rate * dt * .001f, 1e-7);
        EXPECT_NEAR(b.pitch, a.pitch + a.pitch_rate * dt * .001f, 1e-7);
    }
    const auto b = c.tick(105, PIVOT, {});
    EXPECT_TRUE(c.replanned());
    EXPECT_NEAR(b.yaw, a.yaw + a.yaw_rate * .005f, .0001);
    c.observe(106, in);
    c.tick(106, PIVOT, {});
    EXPECT_TRUE(c.replanned());
}
TEST(ManagedAim, RatesMatchContinuousSolutionIncludingAccelerationSpinAndShooterMotion) {
    AimTuning t;
    for (float spin : {0.f, 1.5f, 9.f, -9.f}) {
        auto in = target();
        in.yaw_rate = spin;
        in.velocity = {.3f, 1, .1f};
        in.acceleration = {.2f, -.3f, .2f};
        const Vec3 sv{.2f, -.5f, 0};
        AimPlanner a(t), b(t);
        const auto p = a.plan(0, in, PIVOT, sv);
        in.state_age_s = .001f;
        const auto q = b.plan(1, in, PIVOT + sv * .001f, sv);
        ASSERT_TRUE(p.valid && q.valid);
        ASSERT_EQ(p.panel, q.panel);
        EXPECT_NEAR(wrap_to_pi(q.yaw - p.yaw) / .001f, p.yaw_rate, .004);
        EXPECT_NEAR((q.pitch - p.pitch) / .001f, p.pitch_rate, .004);
    }
}
TEST(ManagedAim, MovingShooterAndMovingPanelIntersectAtImpact) {
    AimTuning tuning;
    tuning.shot_gating_rad_s = std::numeric_limits<float>::infinity();
    for (float spin : {0.f, 2.f, -9.f, 9.f}) {
        auto in = target();
        in.yaw_rate = spin;
        in.velocity = {.3f, 2, 0};
        in.acceleration = {.1f, -.4f, .1f};
        const Vec3 sv{.2f, -1, 0};
        AimPlanner planner(tuning);
        auto p = planner.plan(0, in, PIVOT, sv);
        ASSERT_TRUE(p.valid);
        auto hit = predict_impact(in, p, PIVOT, sv, aligned(p), tuning);
        ASSERT_TRUE(hit) << spin;
        EXPECT_TRUE(hit->inside) << hit->width << " " << hit->height;
        EXPECT_NEAR(hit->width, 0, .001);
        EXPECT_NEAR(hit->height, 0, .001);
        EXPECT_NEAR(hit->time, p.flight_time, .0001);
    }
}
TEST(ManagedAim, FireRequiresCurrentWindowAndCurrentOutline) {
    AimControl c;
    auto in = target();
    c.observe(0, in);
    auto p = c.tick(0, PIVOT, {});
    ASSERT_TRUE(p.valid);
    EXPECT_TRUE(c.can_fire(0, PIVOT, {}, aligned(p)));
    auto miss = aligned(p);
    miss.yaw += .1f;
    EXPECT_FALSE(c.can_fire(0, PIVOT, {}, miss));
    EXPECT_FALSE(c.can_fire(600, PIVOT, {}, aligned(p)));
    int open = 0, closed = 0;
    for (float phase = 0; phase < 1.57f; phase += .001f) {
        AimControl spin;
        in = target();
        in.yaw += phase;
        in.yaw_rate = 9;
        spin.observe(0, in);
        p = spin.tick(0, PIVOT, {});
        if (!p.valid) continue;
        const bool window = shot_window(in, p.panel, p.flight_time, PIVOT, {}, true, {});
        EXPECT_EQ(spin.can_fire(0, PIVOT, {}, {p.yaw, p.pitch, p.yaw_rate, p.pitch_rate}), window);
        if (window)
            ++open;
        else
            ++closed;
    }
    EXPECT_GT(open, 0);
    EXPECT_GT(closed, 0);
}
TEST(ManagedAim, LatencyExtrapolatesPhysicalRateWithYawAndPitchSigns) {
    auto in = target();
    AimPlanner planner;
    const auto p = planner.plan(0, in, PIVOT, {});
    ASSERT_TRUE(p.valid);
    auto measured = aligned(p);
    measured.yaw_rate = .7f;
    measured.pitch_rate = -.4f;
    measured.yaw -= .7f * .020f;
    measured.pitch += .4f * .020f;
    const auto hit = predict_impact(in, p, PIVOT, {}, measured, {});
    ASSERT_TRUE(hit);
    EXPECT_TRUE(hit->inside);
    measured.yaw_rate = -.7f;
    measured.pitch_rate = .4f;
    const auto miss = predict_impact(in, p, PIVOT, {}, measured, {});
    EXPECT_TRUE(!miss || !miss->inside);
}
TEST(ManagedAim, WholeTiltedObliqueOutlineIncludesEdgesAndCornersWithoutMargin) {
    AimTuning t;
    t.ballistics.bore_right = 0;
    for (float angle : {0.f, .65f, -.65f}) {
        auto in = target();
        in.yaw += angle;
        AimPlanner planner(t);
        const auto p = planner.plan(0, in, PIVOT, {});
        ASSERT_TRUE(p.valid);
        const auto panel = panel_at(in, p.panel, 0);
        for (float u : {-.0675f, 0.f, .0675f})
            for (float v : {-.0625f, 0.f, .0625f}) {
                const Vec3 point = panel.center + panel.width * u + panel.height * v;
                const Vec3 d = point - PIVOT;
                const auto b = solve_ballistic(std::hypot(d.x, d.y), d.z, t.ballistics);
                ASSERT_TRUE(b);
                GimbalMotion measured{std::atan2(d.y, d.x), b->pitch, 0, 0};
                const auto hit = predict_impact(in, p, PIVOT, {}, measured, t);
                ASSERT_TRUE(hit);
                EXPECT_NEAR(hit->width, u, .00001);
                EXPECT_NEAR(hit->height, v, .00051);
                // Check physical edge/corner shots just inside float rounding; exact bounds below.
                const Vec3 inset = panel.center + panel.width * (u * .999f) + panel.height * (v * .999f) - PIVOT;
                const float range = std::hypot(inset.x, inset.y);
                const auto edge = solve_ballistic(range, inset.z, t.ballistics);
                ASSERT_TRUE(edge);
                float pitch = edge->pitch;
                for (int i = 0; i < 2; ++i) {
                    const auto sample = ballistic_height(range, pitch, t.ballistics);
                    ASSERT_TRUE(sample);
                    pitch -= (sample->height - inset.z) / sample->pitch_derivative;
                }
                const auto inside = predict_impact(in, p, PIVOT, {}, {std::atan2(inset.y, inset.x), pitch, 0, 0}, t);
                ASSERT_TRUE(inside);
                EXPECT_TRUE(inside->inside) << angle << " " << u << " " << v;
            }
        const Vec3 point = panel.center + panel.width * .0676f;
        const Vec3 d = point - PIVOT;
        const auto b = solve_ballistic(std::hypot(d.x, d.y), d.z, t.ballistics);
        ASSERT_TRUE(b);
        const auto miss = predict_impact(in, p, PIVOT, {}, {std::atan2(d.y, d.x), b->pitch, 0, 0}, t);
        ASSERT_TRUE(miss);
        EXPECT_FALSE(miss->inside);
    }
}
TEST(ManagedAim, TravelLimitedMissFailsButRequestedPitchRemainsUnclamped) {
    auto in = target();
    in.center.z = 2;
    AimControl c;
    c.observe(0, in);
    const auto p = c.tick(0, PIVOT, {});
    ASSERT_TRUE(p.valid);
    EXPECT_LT(p.pitch, -.0873f);
    auto measured = aligned(p);
    measured.pitch = -.0873f;
    EXPECT_FALSE(c.can_fire(0, PIVOT, {}, measured));
    EXPECT_TRUE(c.can_fire(0, PIVOT, {}, aligned(p)));
}
TEST(ManagedAim, BackFaceDegenerateAndNonfiniteShotsFailClosed) {
    auto in = target();
    AimPlanner planner;
    auto p = planner.plan(0, in, PIVOT, {});
    ASSERT_TRUE(p.valid);
    p.panel = (p.panel + 2) % 4;
    EXPECT_FALSE(predict_impact(in, p, PIVOT, {}, aligned(p), {}));
    p.panel = (p.panel + 2) % 4;
    auto measured = aligned(p);
    measured.pitch = PI_F / 2;
    EXPECT_FALSE(predict_impact(in, p, PIVOT, {}, measured, {}));
    measured = aligned(p);
    measured.yaw = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(predict_impact(in, p, PIVOT, {}, measured, {}));
}
TEST(ManagedAim, InterceptExhaustionFailsClosedAndResetDropsPlan) {
    AimControl c;
    auto in = target();
    in.velocity = {100, 0, 0};
    c.observe(0, in);
    EXPECT_FALSE(c.tick(0, PIVOT, {}).valid);
    EXPECT_FALSE(c.can_fire(0, PIVOT, {}, {}));
    c.reset();
    EXPECT_EQ(c.state(), AimControl::State::WAITING);
}
TEST(ChoosePanel, HysteresisAndRelabelingPreserveGeometricIdentity) {
    EXPECT_EQ(choose_panel(0, PI_F, std::nullopt, 0), 2);
    const float held = PI_F / 2 + .6f;
    EXPECT_EQ(choose_panel(.6f, PI_F, held, .5f), 1);
    EXPECT_EQ(choose_panel(.6f - PI_F / 2, PI_F, held, .5f), 2);
    EXPECT_EQ(choose_panel(-PI_F + .02f, PI_F, PI_F - .02f, .2f), 0);
}

TEST(ManagedAim, ExactPanelBoundaryIncludesAllCornersAndRejectsTheNextFloat) {
    for (float u : {-.0675f, .0675f})
        for (float h : {-.0625f, .0625f}) {
            EXPECT_TRUE(inside_panel(u, h));
            EXPECT_FALSE(inside_panel(std::nextafter(u, u * 2), h));
            EXPECT_FALSE(inside_panel(u, std::nextafter(h, h * 2)));
        }
}
TEST(ManagedAim, ControllerBackProjectionDoesNotDoubleLeadTheFiringLatency) {
    auto in = target();
    in.velocity = {0, 4, 0};
    AimControl c;
    c.observe(0, in);
    const auto p = c.tick(0, PIVOT, {});
    ASSERT_TRUE(p.valid);
    EXPECT_TRUE(c.can_fire(0, PIVOT, {}, {p.yaw, p.pitch, p.yaw_rate, p.pitch_rate}));
}

TEST(ManagedAim, ShotGatingCutoutIsStrictSymmetricAndRefreshesWithoutDwell) {
    EXPECT_FLOAT_EQ(AimTuning{}.shot_gating_rad_s, PI_F / 2);
    AimTuning chase, gated;
    chase.shot_gating_rad_s = std::numeric_limits<float>::infinity();
    gated.chase = false;
    for (float sign : {-1.f, 1.f}) {
        AimPlanner automatic;
        for (float rate : {std::nextafter(SHOT_GATING_RAD_S, 0.f), SHOT_GATING_RAD_S,
                           std::nextafter(SHOT_GATING_RAD_S, 3.f), 9.f,
                           SHOT_GATING_RAD_S}) {
            auto in = target();
            in.yaw += .2f;
            in.yaw_rate = sign * rate;
            AimPlanner reference(rate > SHOT_GATING_RAD_S ? gated : chase);
            const auto p = automatic.plan(0, in, PIVOT, {});
            const auto q = reference.plan(0, in, PIVOT, {});
            ASSERT_TRUE(p.valid && q.valid);
            EXPECT_NEAR(p.aim.x, q.aim.x, 1e-5);
            EXPECT_NEAR(p.aim.y, q.aim.y, 1e-5);
            EXPECT_NEAR(p.yaw_rate, q.yaw_rate, 1e-5);
            if (rate > SHOT_GATING_RAD_S) {
                EXPECT_NEAR(p.aim.y, 0, 1e-6);
                EXPECT_NEAR(p.yaw_rate, 0, 1e-6);
            } else {
                EXPECT_GT(std::abs(p.aim.y), .001);
            }
            EXPECT_EQ(shot_window(in, p.panel, p.flight_time, PIVOT, {}, p.spinning, {}),
                      shot_window(in, q.panel, q.flight_time, PIVOT, {}, p.spinning,
                                  rate > SHOT_GATING_RAD_S ? gated : chase));
        }
    }
}

TEST(ManagedAim, FixedAimKeepsImpactAndCrossingGatesForBothSpinDirections) {
    for (float rate : {-9.f, -2.f, 2.f, 9.f}) {
        auto in = target();
        in.yaw_rate = rate;
        AimPlanner planner;
        auto p = planner.plan(0, in, PIVOT, {});
        ASSERT_TRUE(p.valid);
        in.yaw = PI_F - rate * (.020f + p.flight_time);
        AimControl c;
        c.observe(0, in);
        p = c.tick(0, PIVOT, {});
        ASSERT_TRUE(p.valid);
        const GimbalMotion measured{p.yaw, p.pitch, p.yaw_rate, p.pitch_rate};
        EXPECT_TRUE(c.can_fire(0, PIVOT, {}, measured));
        auto miss = measured;
        miss.yaw += .1f;
        EXPECT_FALSE(c.can_fire(0, PIVOT, {}, miss));
        in.yaw += .1f;
        c.observe(1, in);
        p = c.tick(1, PIVOT, {});
        EXPECT_FALSE(c.can_fire(1, PIVOT, {}, {p.yaw, p.pitch, p.yaw_rate, p.pitch_rate}));
    }
}
