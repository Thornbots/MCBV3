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
    for (const auto& [offset, spinning] : {std::pair{0.10f, true}, std::pair{0.25f, true},
            std::pair{0.40f, false}, std::pair{0.25f, false}, std::pair{0.10f, true}}) {
        odo.relocalizeTo(-2.40f - offset, -0.635f);
        command.execute();
        if (spinning) EXPECT_GT(spin(), 1.0f) << offset;
        else EXPECT_LT(std::abs(spin()), 1.0f) << offset;
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
    goal(-5.25f, -0.635f);
    command.execute();
    EXPECT_NEAR(drive.getHostedTargetVelocity().magnitude(), 0, 1e-3f);
}
}  // namespace commands
