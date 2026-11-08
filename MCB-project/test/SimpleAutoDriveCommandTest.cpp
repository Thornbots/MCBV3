// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include <algorithm>
#include <iterator>
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
