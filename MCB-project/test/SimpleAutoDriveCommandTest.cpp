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
            for (int i : {1, 2}) {
                const float x = (blue ? 1.0f : -1.0f) * (4.625f - (i == 1 ? 0.5f : 1.4f));
                EXPECT_NEAR(command.targets[i].position.first, x, 1e-6f);
                EXPECT_NEAR(command.targets[i].position.second, -2.5f, 1e-6f);
            }
        }
    }

    struct FieldWaypoint { float x, y, vx, vy; };

    void checkFieldRoute(Mode mode, bool blue, std::initializer_list<FieldWaypoint> expected,
            std::pair<float, float> initial) {
        team(blue);
        SimpleAutoDriveCommand command(&drivers, &drive, &gimbal, &odo, mode);
        command.initialize();
        ASSERT_EQ(command.targets.size(), expected.size());
        size_t i = 0;
        for (const auto& point : expected) {
            SCOPED_TRACE(i);
            const auto& actual = command.targets[i++];
            EXPECT_NEAR(actual.position.first, point.x, 2e-6f);
            EXPECT_NEAR(actual.position.second, point.y, 2e-6f);
            EXPECT_NEAR(actual.velocity.first, point.vx, 2e-6f);
            EXPECT_NEAR(actual.velocity.second, point.vy, 2e-6f);
        }
        EXPECT_NEAR(command.changedInitialPoint.first, initial.first, 2e-6f);
        EXPECT_NEAR(command.changedInitialPoint.second, initial.second, 2e-6f);
    }

    void checkLegacyFieldCoordinates() {
        // Golden field coordinates from the former red=0/blue=PI route definitions.
        checkFieldRoute(Mode::TEST, false, {
            {-4.625f, 0.0f, 0.0f, 0.0f},
            {-3.625f, 0.0f, 0.0f, 0.0f},
            {-3.625f, 1.0f, 0.0f, 0.0f},
            {-4.625f, 1.0f, 0.0f, 0.0f},
            {-4.625f, 0.0f, 0.0f, 0.0f},
        }, {0.0f, 0.0f});
        checkFieldRoute(Mode::TEST, true, {
            {4.625f, 0.0f, 0.0f, 0.0f},
            {3.625f, 0.0f, 0.0f, 0.0f},
            {3.625f, 1.0f, 0.0f, 0.0f},
            {4.625f, 1.0f, 0.0f, 0.0f},
            {4.625f, 0.0f, 0.0f, 0.0f},
        }, {0.0f, 0.0f});
        checkFieldRoute(Mode::PURDUE2V2, false, {
            {-4.625f, 0.0f, 0.0f, 0.0f},
            {-4.625f, -0.18f, 0.0f, -1.5f},
            {-4.60824f, -0.24f, 0.75f, -1.299f},
            {-4.565f, -0.2838f, 1.299f, -0.75f},
            {-4.505f, -0.3f, 1.5f, 0.0f},
            {-4.4908f, -0.3f, 1.5f, 0.0f},
            {-4.4128f, -0.2922f, 1.47f, 0.29f},
            {-4.3376f, -0.2694f, 1.385f, 0.574f},
            {-4.2686f, -0.2324f, 1.247f, 0.833f},
            {-4.2078f, -0.1828f, 1.06f, 1.06f},
            {-3.925f, 0.1f, 0.0f, 0.0f},
        }, {0.0f, 0.0f});
        checkFieldRoute(Mode::PURDUE2V2, true, {
            {4.625f, 0.0f, 0.0f, 0.0f},
            {4.625f, -1.5f, 0.0f, 0.0f},
            {3.125f, -1.5f, 0.0f, 0.0f},
            {1.125f, 0.5f, 0.0f, 0.0f},
        }, {0.0f, 0.0f});
        checkFieldRoute(Mode::ARCC_RAMP_PATH, false, {
            {-4.625f, 0.0f, 0.0f, 0.0f},
            {-4.03f, 1.833f, 0.0f, 0.0f},
            {-0.565f, 1.833f, 0.0f, 0.0f},
            {0.0f, -0.5f, 0.0f, 0.0f},
        }, {-5.125f, -0.5f});
        checkFieldRoute(Mode::ARCC_RAMP_PATH, true, {
            {4.625f, 0.0f, 0.0f, 0.0f},
            {4.03f, 1.833f, 0.0f, 0.0f},
            {0.565f, 1.833f, 0.0f, 0.0f},
            {0.0f, -0.5f, 0.0f, 0.0f},
        }, {5.125f, -0.5f});
        checkFieldRoute(Mode::ARCC_RAMP_PATH_HYPOTENUSE_ADJUSTED, false, {
            {-4.625f, 0.0f, 0.0f, 0.0f},
            {-4.03f, 1.833f, 0.0f, 0.0f},
            {0.247f, 1.833f, 0.0f, 0.0f},
            {0.812f, -0.5f, 0.0f, 0.0f},
        }, {-5.125f, -0.5f});
        checkFieldRoute(Mode::ARCC_RAMP_PATH_HYPOTENUSE_ADJUSTED, true, {
            {4.625f, 0.0f, 0.0f, 0.0f},
            {4.03f, 1.833f, 0.0f, 0.0f},
            {-0.247f, 1.833f, 0.0f, 0.0f},
            {-0.812f, -0.5f, 0.0f, 0.0f},
        }, {5.125f, -0.5f});
        checkFieldRoute(Mode::ARCC_HALLWAY_PATH, false, {
            {-4.625f, 0.0f, 0.0f, 0.0f},
            {-3.733f, 0.874f, 0.0f, 0.0f},
            {-2.901f, 0.874f, 0.0f, 0.0f},
            {-2.901f, -0.693f, 0.0f, 0.0f},
            {-2.395f, -1.2f, 0.0f, 0.0f},
            {0.0f, -0.885f, 0.0f, 0.0f},
        }, {-5.125f, -0.5f});
        checkFieldRoute(Mode::ARCC_HALLWAY_PATH, true, {
            {4.625f, 0.0f, 0.0f, 0.0f},
            {3.733f, 0.874f, 0.0f, 0.0f},
            {2.901f, 0.874f, 0.0f, 0.0f},
            {2.901f, -0.693f, 0.0f, 0.0f},
            {2.395f, -1.2f, 0.0f, 0.0f},
            {0.0f, -0.885f, 0.0f, 0.0f},
        }, {5.125f, -0.5f});
        checkFieldRoute(Mode::ARCC_ROUGH_PATH, false, {
            {-4.625f, 0.0f, 0.0f, 0.0f},
            {-4.125f, -2.5f, 0.0f, 0.0f},
            {-3.225f, -2.5f, 0.0f, 0.0f},
            {0.0f, 0.5f, 0.0f, 0.0f},
        }, {-5.125f, 0.5f});
        checkFieldRoute(Mode::ARCC_ROUGH_PATH, true, {
            {4.625f, 0.0f, 0.0f, 0.0f},
            {4.125f, -2.5f, 0.0f, 0.0f},
            {3.225f, -2.5f, 0.0f, 0.0f},
            {0.0f, 0.5f, 0.0f, 0.0f},
        }, {5.125f, 0.5f});
    }

    void checkBootAxes(bool blue) {
        team(blue);
        EXPECT_NEAR(odo.getStartYaw(), 1.57079632679f, 1e-6f);
        const float startX = blue ? 4.625f : -4.625f;
        const auto point = odo.fromStart(2.0f, 0.7f);
        EXPECT_NEAR(point.getX(), startX - 0.7f, 1e-6f);
        EXPECT_NEAR(point.getY(), 2.0f, 1e-6f);
        hosted::sensors.podX = 0.3f;  // pod right is field +x
        hosted::sensors.podY = 0.5f;  // pod forward is field +y
        hosted::sensors.podVx = -0.2f;
        hosted::sensors.podVy = 0.4f;
        EXPECT_NEAR(odo.getAimX(), startX + 0.3f, 1e-6f);
        EXPECT_NEAR(odo.getAimY(), 0.5f, 1e-6f);
        EXPECT_NEAR(odo.getXVel(), -0.2f, 1e-6f);
        EXPECT_NEAR(odo.getYVel(), 0.4f, 1e-6f);
        hosted::sensors = {};
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

TEST_F(SimpleAutoDriveCommandTest, BothTeamsBootAlongFieldPositiveY) {
    checkBootAxes(false);
    checkBootAxes(true);
}

TEST_F(SimpleAutoDriveCommandTest, EveryRoutePreservesLegacyFieldCoordinatesAndVelocities) {
    checkLegacyFieldCoordinates();
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
