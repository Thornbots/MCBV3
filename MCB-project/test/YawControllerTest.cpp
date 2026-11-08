// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include "subsystems/gimbal/controllers/YawController.hpp"
#include "subsystems/gimbal/controllers/AngleSetpointRate.hpp"

TEST(YawPrediction, CoastingUsesConfiguredViscousAndStaticFriction) {
    subsystems::YawController controller;
    float yaw = 0, rate = 0.5;
    controller.estimateState(&yaw, &rate, 0, 0);
    const double decay = 1 - C * DT / J;
    EXPECT_NEAR(rate, .5 * std::pow(decay, Q_SIZE), 1e-6);
    EXPECT_GT(yaw, 0);
    EXPECT_LT(rate, .5);
}

TEST(YawPrediction, CurrentFeedforwardUsesGearedOutputTorque) {
    subsystems::YawController controller;
    const float output = controller.calculate(0, 0, 0, .1, 0, .002);
    const double current = output / .8192;
    float yaw = 0, rate = 0;
    controller.estimateYawPos(yaw, rate, 0);
    EXPECT_GT(output, 0);
    EXPECT_NEAR(rate, current * KT * RATIO * DT / J, 1e-6);
    EXPECT_NEAR(yaw, current * KT * RATIO * DT * DT / J, 1e-6);
}

TEST(YawSetpointRate, SparseMovingPointsPreserveSpeedWithoutControlTickSpikes) {
    subsystems::AngleSetpointRate estimator;
    float angle = 0, sum = 0, peak = 0;
    estimator.update(angle, .001f);
    for (int tick = 1; tick <= 2000; ++tick) {
        if (tick % 25 == 0) angle += .05f; // 40 Hz, 2 rad/s
        const float rate = estimator.update(angle, .001f);
        if (tick > 1000) { sum += rate; peak = std::max(peak, rate); }
    }
    EXPECT_NEAR(sum / 1000, 2, .01);
    EXPECT_LT(peak, 4); // raw one-tick derivative would be 50 rad/s
}
TEST(YawSetpointRate, WrapAndTargetHandoffDoNotRequestTheLongTurn) {
    subsystems::AngleSetpointRate estimator;
    estimator.update(3.13f, .001f);
    EXPECT_GT(estimator.update(-3.13f, .001f), 0);
    EXPECT_LT(estimator.update(-3.13f, .001f), 1);
    EXPECT_EQ(estimator.update(0, .001f), 0);
    estimator.reset();
    EXPECT_EQ(estimator.update(2, .001f), 0);
}
