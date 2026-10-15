// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>

#include "util/HitEventState.hpp"
#include "util/HitOrientation.hpp"

TEST(HitTrackerTest, ReportsPositiveCounterclockwiseRadiansRelativeToCurrentChassisYaw)
{
    constexpr float pi = 3.14159265358979323846f;
    constexpr float headWorldYaw = 1.2f;
    constexpr float yawEncoder = 0.4f;
    constexpr float chassisWorldYaw = headWorldYaw - yawEncoder;
    const float hitWorldYaw = hit_orientation::worldYaw(headWorldYaw, yawEncoder, 1);

    EXPECT_NEAR(hitWorldYaw, chassisWorldYaw + pi / 2.0f, 1e-6f);
    EXPECT_NEAR(hit_orientation::relativeYaw(hitWorldYaw, chassisWorldYaw), pi / 2.0f, 1e-6f);
    EXPECT_NEAR(hit_orientation::relativeYaw(hitWorldYaw, chassisWorldYaw + 0.3f), pi / 2.0f - 0.3f, 1e-6f);
}

TEST(HitTrackerTest, RightPanelIsNegativeAndNoCurrentHitUsesPlaceholder)
{
    constexpr float pi = 3.14159265358979323846f;
    constexpr float headWorldYaw = 2.0f;
    constexpr float yawEncoder = 0.5f;
    constexpr float chassisWorldYaw = headWorldYaw - yawEncoder;
    const float hitWorldYaw = hit_orientation::worldYaw(headWorldYaw, yawEncoder, 3);

    EXPECT_NEAR(hitWorldYaw, chassisWorldYaw + 3.0f * pi / 2.0f, 1e-6f);
    EXPECT_NEAR(hit_orientation::relativeYaw(hitWorldYaw, chassisWorldYaw), -pi / 2.0f, 1e-6f);
}

TEST(HitTrackerTest, FrontAndBackPanelsAndWrappedYawHaveExpectedBearings)
{
    constexpr float pi = 3.14159265358979323846f;
    constexpr float headWorldYaw = 0.7f;
    constexpr float yawEncoder = 0.2f;
    constexpr float chassisWorldYaw = headWorldYaw - yawEncoder;
    const float frontWorldYaw = hit_orientation::worldYaw(headWorldYaw, yawEncoder, 0);
    const float backWorldYaw = hit_orientation::worldYaw(headWorldYaw, yawEncoder, 2);

    EXPECT_NEAR(hit_orientation::relativeYaw(frontWorldYaw, chassisWorldYaw), 0.0f, 1e-6f);
    EXPECT_NEAR(hit_orientation::relativeYaw(backWorldYaw, chassisWorldYaw), pi, 1e-6f);
    EXPECT_NEAR(hit_orientation::relativeYaw(-pi + 0.1f, pi - 0.1f), 0.2f, 1e-6f);
}

TEST(HitEventStateTest, KeepsLatestWorldBearingUntilRefSysConsumesIt)
{
    constexpr float pi = 3.14159265358979323846f;
    HitEventState state;
    constexpr float headWorldYaw = 1.2f;
    constexpr float yawEncoder = 0.4f;
    constexpr float chassisWorldYaw = headWorldYaw - yawEncoder;
    const float hitWorldYaw = hit_orientation::worldYaw(headWorldYaw, yawEncoder, 1);

    EXPECT_FALSE(state.hitThisControlCycle());
    EXPECT_FLOAT_EQ(state.hitWorldYawThisControlCycle(), hit_orientation::NO_HIT_ANGLE);
    EXPECT_FLOAT_EQ(state.angleForRefSys(chassisWorldYaw), hit_orientation::NO_HIT_ANGLE);

    state.record(hitWorldYaw);
    EXPECT_TRUE(state.hitThisControlCycle());
    EXPECT_FLOAT_EQ(state.hitWorldYawThisControlCycle(), hitWorldYaw);
    EXPECT_NEAR(state.angleForRefSys(chassisWorldYaw), pi / 2.0f, 1e-6f);

    for (int cycle = 0; cycle < 100; ++cycle) {
        state.beginControlCycle();
    }

    EXPECT_FALSE(state.hitThisControlCycle());
    EXPECT_FLOAT_EQ(state.hitWorldYawThisControlCycle(), hit_orientation::NO_HIT_ANGLE);
    EXPECT_NEAR(state.angleForRefSys(chassisWorldYaw + 0.3f), pi / 2.0f - 0.3f, 1e-6f);

    const float latestWorldYaw = hit_orientation::worldYaw(2.0f, 0.5f, 3);
    state.record(latestWorldYaw);
    EXPECT_NEAR(state.angleForRefSys(1.0f), hit_orientation::relativeYaw(latestWorldYaw, 1.0f), 1e-6f);
    state.consumeReport();
    EXPECT_FLOAT_EQ(state.angleForRefSys(1.0f), hit_orientation::NO_HIT_ANGLE);
    EXPECT_TRUE(state.hitThisControlCycle());
    EXPECT_FLOAT_EQ(state.hitWorldYawThisControlCycle(), latestWorldYaw);

    state.beginControlCycle();
    EXPECT_FALSE(state.hitThisControlCycle());
}
