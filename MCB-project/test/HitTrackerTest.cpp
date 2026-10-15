// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>

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

TEST(HitTrackerTest, RightPanelIsNegativeRelativeToChassisYaw)
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
