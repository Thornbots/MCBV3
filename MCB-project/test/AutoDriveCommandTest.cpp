// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include <algorithm>
#include <iterator>
#include <cmath>
#include <cstring>
#include <limits>
#include "subsystems/jetson/AutoDriveCommand.hpp"
#include "hosted/clock.hpp"
#include "hosted/uart.hpp"
#include "robots/sentry/SentryHardware.hpp"

namespace commands {
class AutoDriveCommandTest : public ::testing::Test {
protected:
    src::Drivers drivers;
    robots::SentryHardware hardware{&drivers};
    subsystems::DrivetrainSubsystem drive{&drivers, &hardware.driveMotor1,
        &hardware.driveMotor2, &hardware.driveMotor3, &hardware.driveMotor4};
    subsystems::GimbalSubsystem gimbal{&drivers, &hardware.yawMotor, &hardware.pitchMotor};
    subsystems::OdometrySubsystem odo{&drivers, &hardware.odoMotor};

    void SetUp() override {
        hosted::timeMs = 0;
        hosted::sensors = {};
        drivers.remote.initialize();
        const uint8_t remote[18] = {0, 4, 32, 0, 1, 88, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4};
        hosted::queueUart(tap::communication::serial::Uart::Uart3, remote, sizeof(remote));
        drivers.remote.read();
        ASSERT_TRUE(drivers.remote.isConnected());
        tap::communication::serial::RefSerial::ReceivedSerialMessage game{};
        std::fill(std::begin(game.data), std::end(game.data), 0);
        game.messageType = tap::communication::serial::RefSerial::REF_MESSAGE_TYPE_GAME_STATUS;
        game.header.dataLength = 11;
        game.data[0] = static_cast<uint8_t>(RefSerialData::Rx::GameStage::IN_GAME) << 4;
        drivers.refSerial.messageReceiveCallback(game);
    }

    void TearDown() override { hosted::sensors = {}; }

    void team(bool blue, bool returning = false) {
        tap::communication::serial::RefSerial::ReceivedSerialMessage robot{};
        std::fill(std::begin(robot.data), std::end(robot.data), 0);
        robot.messageType = tap::communication::serial::RefSerial::REF_MESSAGE_TYPE_ROBOT_STATUS;
        robot.header.dataLength = 13;
        robot.data[0] = blue ? 107 : 7;  // sentry IDs
        robot.data[2] = returning ? 50 : 100;
        robot.data[4] = 100;
        drivers.refSerial.messageReceiveCallback(robot);
        odo.refresh();
        ASSERT_EQ(drivers.refSerial.getRobotData().currentHp, returning ? 50 : 100);
        ASSERT_EQ(drivers.refSerial.getRobotData().maxHp, 100);
    }

    void goal(float x, float y) {
        communication::UARTCommunication::ReceivedSerialMessage message{};
        const subsystems::NavGoal goal{x, y};
        message.messageType = subsystems::NAV_GOAL;
        message.header.dataLength = sizeof(goal);
        std::memcpy(message.data, &goal, sizeof(goal));
        drivers.uart.messageReceiveCallback(message);
    }

    void stage(RefSerialData::Rx::GameStage gameStage) {
        tap::communication::serial::RefSerial::ReceivedSerialMessage game{};
        std::fill(std::begin(game.data), std::end(game.data), 0);
        game.messageType = tap::communication::serial::RefSerial::REF_MESSAGE_TYPE_GAME_STATUS;
        game.header.dataLength = 11;
        game.data[0] = static_cast<uint8_t>(RefSerialData::Rx::GameType::ROBOMASTER_RMUL_3V3) |
                       static_cast<uint8_t>(gameStage) << 4;
        drivers.refSerial.messageReceiveCallback(game);
    }

    float spin() const { return drive.getHostedTargetVelocity().getRotation(); }
    float translation() const { return drive.getHostedTargetVelocity().magnitude(); }

    void chassisYaw(float yaw, float rate) {
        hosted::sensors.yaw = yaw;
        hosted::sensors.yawRate = rate;
        gimbal.refresh();
        gimbal.clearBuildup();
    }

    void yawMotorFeedback(uint16_t rotorTicks) {
        modm::can::Message feedback(hardware.yawMotor.getMotorIdentifier(), 8);
        std::fill(std::begin(feedback.data), std::end(feedback.data), 0);
        feedback.data[0] = rotorTicks >> 8;
        feedback.data[1] = rotorTicks & 0xff;
        hardware.yawMotor.processMessage(feedback);
        gimbal.refresh();
        ++hosted::timeMs;
    }
};

TEST_F(AutoDriveCommandTest, RawStartsAtZeroForBothTeamsAndNeverReanchors) {
    for (bool blue : {false, true}) {
        team(blue);
        EXPECT_FLOAT_EQ(odo.getAimX(), 0);
        EXPECT_FLOAT_EQ(odo.getAimY(), 0);
        EXPECT_FALSE(odo.isNavigationLocalized());
    }
    hosted::sensors.podY = 0.5f;
    const float x = odo.getAimX();
    const float y = odo.getAimY();
    EXPECT_NEAR(x, 0, 1e-6f);
    EXPECT_NEAR(y, 0.5f, 1e-6f);
    odo.relocalizeTo(2, 3);
    team(false);
    EXPECT_FLOAT_EQ(odo.getAimX(), x);
    EXPECT_FLOAT_EQ(odo.getAimY(), y);
    EXPECT_FLOAT_EQ(odo.getX(), 2);
    EXPECT_FLOAT_EQ(odo.getY(), 3);
    EXPECT_TRUE(odo.isNavigationLocalized());
    hosted::sensors.podY = 0.75f;
    EXPECT_NEAR(odo.getAimX(), x, 1e-6f);
    EXPECT_NEAR(odo.getAimY(), y + 0.25f, 1e-6f);
}


TEST_F(AutoDriveCommandTest, BothTeamsKeepBootHeadingPodAxesAndVelocity) {
    for (bool blue : {false, true}) {
        team(blue);
        EXPECT_NEAR(odo.getStartYaw(), 1.5707963267948966f, 1e-7f);
        hosted::sensors.podX = 0.3f;
        hosted::sensors.podY = 0.5f;
        hosted::sensors.podVx = -0.2f;
        hosted::sensors.podVy = 0.4f;
        EXPECT_NEAR(odo.getAimX(), 0.3f, 1e-6f);
        EXPECT_NEAR(odo.getAimY(), 0.5f, 1e-6f);
        EXPECT_NEAR(odo.getXVel(), -0.2f, 1e-6f);
        EXPECT_NEAR(odo.getYVel(), 0.4f, 1e-6f);
    }
}


TEST_F(AutoDriveCommandTest, NavGoalsBeforeLocalizationAreDiscarded) {
    subsystems::JetsonSubsystem jetson(&drivers, &gimbal, &odo);
    AutoDriveCommand command(&drivers, &drive, &gimbal, &jetson, &odo);
    command.initialize();
    goal(4, 3);
    command.execute();
    EXPECT_FLOAT_EQ(drive.getHostedTargetVelocity().magnitude(), 0);
    communication::UARTCommunication::ReceivedSerialMessage fixMessage{};
    const subsystems::Relocalize fix{1, 1};
    fixMessage.messageType = subsystems::RELOCALIZE;
    fixMessage.header.dataLength = sizeof(fix);
    std::memcpy(fixMessage.data, &fix, sizeof(fix));
    drivers.uart.messageReceiveCallback(fixMessage);
    jetson.checkApplyRelocalize();
    ASSERT_TRUE(odo.isNavigationLocalized());
    command.execute();
    EXPECT_FLOAT_EQ(drive.getHostedTargetVelocity().magnitude(), 0);
    goal(4, 3);
    command.execute();
    EXPECT_GT(drive.getHostedTargetVelocity().magnitude(), 0);
}

TEST_F(AutoDriveCommandTest, TravelHoldsAQuarterTurnAndSpinsOnlyAtTheGoal) {
    subsystems::JetsonSubsystem jetson(&drivers, &gimbal, &odo);
    AutoDriveCommand command(&drivers, &drive, &gimbal, &jetson, &odo);
    command.initialize();
    odo.relocalizeTo(-5.25f, -0.635f);
    goal(-2.40f, -0.635f);  // the Starting Zone doorway leg
    command.execute();
    EXPECT_GT(drive.getHostedTargetVelocity().magnitude(), 0);
    EXPECT_LT(std::abs(spin()), 1.0f) << "squared up through the doorway, not spinning";
    // Within SPIN_START_RADIUS: spin; between the radii: keep spinning (hysteresis).
    for (const auto& [offset, spinning] : {std::pair{0.01f, true}, std::pair{0.25f, true},
            std::pair{0.40f, false}, std::pair{0.25f, false}, std::pair{0.01f, true}}) {
        odo.relocalizeTo(-2.40f - offset, -0.635f);
        goal(-2.40f, -0.635f);  // The publisher repeats the same goal at 10 Hz.
        command.execute();
        if (spinning) {
            EXPECT_GT(spin(), 1.0f) << offset;
            EXPECT_FLOAT_EQ(translation(), 0);
        } else {
            EXPECT_LT(std::abs(spin()), 1.0f) << offset;
            EXPECT_GT(translation(), 0);
        }
    }
}

TEST_F(AutoDriveCommandTest, TravelRequiresBothQuarterTurnHeadingAndLowChassisYawRate) {
    subsystems::JetsonSubsystem jetson(&drivers, &gimbal, &odo);
    AutoDriveCommand command(&drivers, &drive, &gimbal, &jetson, &odo);
    command.initialize();
    odo.relocalizeTo(-5.25f, -3.0f);
    goal(-5.25f, -3.0f);
    command.execute();
    ASSERT_GT(spin(), 1.0f);

    goal(-5.25f, -0.635f);
    chassisYaw(0, 8.0f);
    command.execute();
    EXPECT_FLOAT_EQ(translation(), 0);
    EXPECT_LT(std::abs(spin()), 1.0f);

    chassisYaw(0.3f, 0);
    command.execute();
    EXPECT_FLOAT_EQ(translation(), 0);
    EXPECT_LT(spin(), -1.0f);

    chassisYaw(0, 0);
    command.execute();
    EXPECT_GT(translation(), 0);

    chassisYaw(0, -2.0f);
    command.execute();
    EXPECT_FLOAT_EQ(translation(), 0);
}

TEST_F(AutoDriveCommandTest, SpinWaitsForPrecisionArrivalAndBraking) {
    subsystems::JetsonSubsystem jetson(&drivers, &gimbal, &odo);
    AutoDriveCommand command(&drivers, &drive, &gimbal, &jetson, &odo);
    command.initialize();
    odo.relocalizeTo(-5.25f, -0.695f);
    goal(-5.25f, -0.635f);
    command.execute();
    EXPECT_GT(translation(), 0);
    EXPECT_NEAR(spin(), 0, 1e-3f);

    odo.relocalizeTo(-5.25f, -0.645f);
    hosted::sensors.podVy = 0.4f;
    command.execute();
    EXPECT_GT(translation(), 0);
    EXPECT_NEAR(spin(), 0, 1e-3f);

    hosted::sensors.podVy = 0;
    command.execute();
    EXPECT_FLOAT_EQ(translation(), 0);
    EXPECT_GT(spin(), 1.0f);

    goal(-5.25f, -0.80f);  // A new goal inside the spin stop radius still exits spin.
    command.execute();
    EXPECT_GT(translation(), 0);
    EXPECT_NEAR(spin(), 0, 1e-3f);
}

TEST_F(AutoDriveCommandTest, NewLegWaitsForTheCornerToSettle) {
    subsystems::JetsonSubsystem jetson(&drivers, &gimbal, &odo);
    AutoDriveCommand command(&drivers, &drive, &gimbal, &jetson, &odo);
    command.initialize();
    odo.relocalizeTo(-5.25f, -0.68f);
    goal(-5.25f, -0.635f);
    command.execute();
    ASSERT_LT(translation(), 0.15f);

    goal(-2.4f, -0.635f);
    command.execute();
    EXPECT_LT(translation(), 0.15f);
    EXPECT_NEAR(spin(), 0, 1e-3f);

    odo.relocalizeTo(-5.25f, -0.64f);
    hosted::sensors.podVy = 0.4f;
    goal(-2.4f, -0.635f);
    command.execute();
    EXPECT_LT(translation(), 0.15f);
    EXPECT_NEAR(spin(), 0, 1e-3f);

    hosted::sensors.podVy = 0;
    command.execute();
    EXPECT_GT(translation(), 1.0f);
}

TEST_F(AutoDriveCommandTest, ReversingToThePreviousGoalLeavesTheCornerImmediately) {
    subsystems::JetsonSubsystem jetson(&drivers, &gimbal, &odo);
    AutoDriveCommand command(&drivers, &drive, &gimbal, &jetson, &odo);
    command.initialize();
    odo.relocalizeTo(-5.25f, -2.5f);
    goal(-5.25f, -3.0f);
    command.execute();
    goal(-5.25f, -0.635f);
    command.execute();

    odo.relocalizeTo(-5.25f, -0.68f);
    goal(-5.25f, -3.0f);
    command.execute();
    EXPECT_GT(translation(), 1.0f);
    EXPECT_NEAR(spin(), 0, 1e-3f);
}

TEST_F(AutoDriveCommandTest, HostedYawHomesAcrossTheRotorWrapAndKeepsTrackingTurns) {
    constexpr uint16_t ticksPerTurn = 8192;
    constexpr float radiansPerTick = 2 * PI / (ticksPerTurn * YAW_TOTAL_RATIO);
    yawMotorFeedback(ticksPerTurn - 4);
    yawMotorFeedback(ticksPerTurn - 4);
    EXPECT_NEAR(gimbal.getYawEncoderValue(), -4 * radiansPerTick, 1e-5f);

    for (uint16_t ticks = 1024; ticks <= 2 * ticksPerTurn; ticks += 1024) {
        yawMotorFeedback(ticks % ticksPerTurn);
        EXPECT_NEAR(gimbal.getYawEncoderValue(), ticks * radiansPerTick, 1e-5f);
    }
}

TEST_F(AutoDriveCommandTest, CountdownSpinsInPlaceAndPreGameHolds) {
    subsystems::JetsonSubsystem jetson(&drivers, &gimbal, &odo);
    AutoDriveCommand command(&drivers, &drive, &gimbal, &jetson, &odo);
    command.initialize();
    odo.relocalizeTo(-5.25f, -3.0f);
    stage(RefSerialData::Rx::GameStage::COUNTDOWN);
    goal(-5.25f, -0.635f);
    command.execute();
    EXPECT_GT(spin(), 1.0f);
    EXPECT_NEAR(drive.getHostedTargetVelocity().magnitude(), 0, 1e-3f);
    stage(RefSerialData::Rx::GameStage::PREMATCH);
    command.execute();
    EXPECT_FLOAT_EQ(spin(), 0);
    EXPECT_NEAR(drive.getHostedTargetVelocity().magnitude(), 0, 1e-3f);
    stage(RefSerialData::Rx::GameStage::IN_GAME);
    command.execute();
    EXPECT_GT(translation(), 0);
}
}  // namespace commands
