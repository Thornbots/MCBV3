// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <cmath>
#include <gtest/gtest.h>
#include "subsystems/gimbal/controllers/YawController.hpp"

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
