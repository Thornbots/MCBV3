// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include <algorithm>
#include <iterator>
#include <cstring>
#include <limits>
#include "subsystems/jetson/AutoDriveCommand.hpp"
#include "hosted/clock.hpp"
#include "hosted/uart.hpp"
#include "robots/sentry/SentryHardware.hpp"
#include "subsystems/drivetrain/SimpleAutoDriveCommand.hpp"

namespace commands {
class SimpleAutoDriveCommandTest : public ::testing::Test {
protected:
    src::Drivers drivers;
    robots::SentryHardware hardware{&drivers};
    subsystems::DrivetrainSubsystem drive{&drivers, &hardware.driveMotor1,
        &hardware.driveMotor2, &hardware.driveMotor3, &hardware.driveMotor4};
    subsystems::GimbalSubsystem gimbal{&drivers, &hardware.yawMotor, &hardware.pitchMotor};
    subsystems::OdometrySubsystem odo{&drivers, &hardware.odoMotor};
    using Mode = SimpleAutoDriveCommand::TargetMode;

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

    int index(const SimpleAutoDriveCommand& command) { return command.targetIndex; }
    bool positionFinished(const SimpleAutoDriveCommand& command) {
        return command.positionCommand.isFinished();
    }

    void checkRoute(Mode mode, bool blue) {
        team(blue);
        SimpleAutoDriveCommand command(&drivers, &drive, &gimbal, &odo, mode);
        command.initialize();
        for (size_t i = 0; i < command.targets.size(); ++i) {
            const bool wall = mode == Mode::ARCC_ROUGH_PATH && (i == 1 || i == 2);
            EXPECT_FLOAT_EQ(command.targets[i].tolerance, wall ? 0.2f : 0.5f) << i;
        }
        if (mode == Mode::ARCC_ROUGH_PATH) {
            const float left = blue ? 2.5f : -2.5f;
            for (int i : {1, 2}) {
                const auto expected = odo.fromStart(i == 1 ? 0.5f : 1.4f, left);
                EXPECT_FLOAT_EQ(command.targets[i].position.first, expected.getX());
                EXPECT_FLOAT_EQ(command.targets[i].position.second, expected.getY());
            }
        }
    }

    void checkTraversal(bool backwards) {
        team(false, backwards);
        SimpleAutoDriveCommand command(&drivers, &drive, &gimbal, &odo, Mode::ARCC_ROUGH_PATH);
        command.initialize();
        // Switch between waypoint tolerances in both traversal orders.
        for (int step = 0; step < 4; ++step) {
            const int i = backwards ? 3 - step : step;
            command.targetIndex = i;
            command.direction = backwards ? -1 : 1;
            command.positionCommand.targetPosition = {100, 100, 0};  // no early advance
            const auto position = command.targets[i].position;
            odo.relocalizeTo(position.first + 0.3f, position.second);
            command.execute();
            EXPECT_EQ(command.targetIndex, i);
            EXPECT_EQ(command.positionCommand.isFinished(), i == 0 || i == 3) << i;
            odo.relocalizeTo(position.first + 0.1f, position.second);
            command.positionCommand.execute();
            EXPECT_TRUE(command.positionCommand.isFinished()) << i;
            command.execute();
            EXPECT_EQ(command.targetIndex, backwards ? std::max(0, i - 1) : std::min(3, i + 1));
            const float nextTolerance = command.targets[command.targetIndex].tolerance;
            EXPECT_FLOAT_EQ(nextTolerance, command.targetIndex == 1 || command.targetIndex == 2 ? 0.2f : 0.5f);
        }
    }
};

TEST_F(SimpleAutoDriveCommandTest, RawStartsAtZeroForBothTeamsAndNeverReanchors) {
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

TEST_F(SimpleAutoDriveCommandTest, BothTeamsKeepBootHeadingPodAxesAndVelocity) {
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

TEST_F(SimpleAutoDriveCommandTest, BothTeamsRoutesStillRunInwardToFieldCenter) {
    for (bool blue : {false, true}) {
        team(blue);
        const auto center = odo.fromStart(4.625f, 0);
        EXPECT_NEAR(center.getX(), 0, 1e-5f);
        EXPECT_NEAR(center.getY(), 0, 1e-5f);
        const auto leg = odo.fromStart(0.5f, blue ? 2.5f : -2.5f);
        EXPECT_NEAR(leg.getX(), blue ? 4.125f : -4.125f, 1e-5f);
        EXPECT_NEAR(leg.getY(), -2.5f, 1e-5f);
    }
}

TEST_F(SimpleAutoDriveCommandTest, RouteCannotAdvanceOrDriveBeforeValidRelocalize) {
    team(false);
    SimpleAutoDriveCommand command(&drivers, &drive, &gimbal, &odo, Mode::ARCC_ROUGH_PATH);
    command.initialize();
    drive.setTargetTranslation({1, 0, 1}, false);
    for (float invalid : {std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::infinity()}) {
        odo.relocalizeTo(invalid, 0);
        odo.relocalizeTo(0, invalid);
        command.execute();
        EXPECT_FALSE(odo.isNavigationLocalized());
        EXPECT_EQ(index(command), 0);
        EXPECT_FALSE(positionFinished(command));
        EXPECT_FLOAT_EQ(drive.getHostedTargetVelocity().magnitude(), 0);
    }
    const auto start = odo.fromStart(0, 0);
    odo.relocalizeTo(start.getX(), start.getY());
    command.execute();
    command.execute();
    EXPECT_EQ(index(command), 1);
    EXPECT_GT(drive.getHostedTargetVelocity().magnitude(), 0);
    const float navX = odo.getX();
    odo.relocalizeTo(std::numeric_limits<float>::quiet_NaN(), 0);
    EXPECT_TRUE(odo.isNavigationLocalized());
    EXPECT_FLOAT_EQ(odo.getX(), navX);
}

TEST_F(SimpleAutoDriveCommandTest, DirectPositionCommandStopsUntilLocalized) {
    MoveToPositionCommand command(&drivers, &drive, &gimbal, &odo, {4, 3, 0}, {0, 0});
    command.inputVelocity = {1, 1, 1};
    drive.setTargetTranslation({1, 1, 1}, false);
    command.execute();
    EXPECT_FLOAT_EQ(drive.getHostedTargetVelocity().magnitude(), 0);
    EXPECT_FALSE(command.isFinished());
    command.end(true);
    EXPECT_FLOAT_EQ(drive.getHostedTargetVelocity().magnitude(), 0);
    odo.relocalizeTo(0, 0);
    command.execute();
    EXPECT_GT(drive.getHostedTargetVelocity().magnitude(), 0);
}

TEST_F(SimpleAutoDriveCommandTest, NavGoalsBeforeLocalizationAreDiscarded) {
    subsystems::JetsonSubsystem jetson(&drivers, &gimbal, &odo);
    AutoDriveCommand command(&drivers, &drive, &gimbal, &jetson, &odo);
    command.initialize();
    auto goal = [&] {
        communication::UARTCommunication::ReceivedSerialMessage message{};
        subsystems::NavGoal goal{4, 3};
        message.messageType = subsystems::NAV_GOAL;
        message.header.dataLength = sizeof(goal);
        std::memcpy(message.data, &goal, sizeof(goal));
        drivers.uart.messageReceiveCallback(message);
    };
    goal();
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
    goal();
    command.execute();
    EXPECT_GT(drive.getHostedTargetVelocity().magnitude(), 0);
}

TEST_F(SimpleAutoDriveCommandTest, OnlyRoughWallWaypointsTightenForEitherTeam) {
    for (bool blue : {false, true}) {
        for (Mode mode : {Mode::TEST, Mode::PURDUE2V2, Mode::ARCC_HALLWAY_PATH,
                Mode::ARCC_ROUGH_PATH, Mode::ARCC_RAMP_PATH,
                Mode::ARCC_RAMP_PATH_HYPOTENUSE_ADJUSTED}) checkRoute(mode, blue);
    }
}

TEST_F(SimpleAutoDriveCommandTest, OutboundAppliesAndRestoresTolerance) {
    checkTraversal(false);
}

TEST_F(SimpleAutoDriveCommandTest, ReturnAppliesAndRestoresTolerance) {
    checkTraversal(true);
}
}  // namespace commands
