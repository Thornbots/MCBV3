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
    const auto& targets(const SimpleAutoDriveCommand& command) { return command.targets; }
    bool positionFinished(const SimpleAutoDriveCommand& command) {
        return command.positionCommand.isFinished();
    }

    bool holding(const SimpleAutoDriveCommand& command) { return command.positionCommand.holdQuarterTurn; }
    void setIndex(SimpleAutoDriveCommand& command, int i) { command.targetIndex = i; }
    // ARCC_S2_3V3 tolerances by index: start, doorway approach, doorway exit, centre.
    static float s2Tolerance(size_t i) { return i == 1 ? 0.05f : i == 2 ? 0.2f : 0.5f; }

    void checkRoute(Mode mode, bool blue) {
        team(blue);
        SimpleAutoDriveCommand command(&drivers, &drive, &gimbal, &odo, mode);
        command.initialize();
        odo.relocalizeTo(blue ? 5.25f : -5.25f, -3.0f);
        command.execute();
        const bool s2 = mode == Mode::ARCC_S2_3V3;
        for (size_t i = 0; i < command.targets.size(); ++i) {
            EXPECT_FLOAT_EQ(command.targets[i].tolerance, s2 ? s2Tolerance(i) : 0.5f) << i;
            EXPECT_EQ(command.targets[i].square, s2 && (i == 1 || i == 2)) << i;
        }
    }

    void checkTraversal(bool backwards) {
        team(false, backwards);
        SimpleAutoDriveCommand command(&drivers, &drive, &gimbal, &odo, Mode::ARCC_S2_3V3);
        command.initialize();
        odo.relocalizeTo(-5.25f, -3.0f);
        command.execute();
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
            odo.relocalizeTo(position.first + 0.04f, position.second);
            command.positionCommand.execute();
            EXPECT_TRUE(command.positionCommand.isFinished()) << i;
            command.execute();
            const int next = command.targetIndex;
            EXPECT_EQ(next, backwards ? std::max(0, i - 1) : std::min(3, i + 1));
            EXPECT_FLOAT_EQ(command.targets[next].tolerance, s2Tolerance(next));
            // Square on every leg touching a doorway waypoint; spin when parked at either end.
            const bool parkedAtEnd = (backwards ? i == 0 : i == 3);
            EXPECT_EQ(command.positionCommand.holdQuarterTurn,
                !parkedAtEnd && (backwards ? next <= 2 : next == 1 || next == 2)) << i;
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
        SimpleAutoDriveCommand command(&drivers, &drive, &gimbal, &odo, Mode::SPOOKY_SCRIM_2V2);
        command.initialize();
        odo.relocalizeTo(blue ? 4.625f : -4.625f, 0);
        command.execute();
        const float sign = blue ? 1.0f : -1.0f;
        const float field[][2] = {{3.50f, -1.75f}, {2.19f, -1.76f}, {0.0f, 0.0f}};
        ASSERT_EQ(targets(command).size(), 4u);  // [0] is the start, added in the constructor
        for (size_t i = 0; i < 3; ++i) {
            EXPECT_NEAR(targets(command)[i + 1].position.first, sign * field[i][0], 1e-5f) << i;
            EXPECT_NEAR(targets(command)[i + 1].position.second, field[i][1], 1e-5f) << i;
        }
    }
}

TEST_F(SimpleAutoDriveCommandTest, LateBlueRefereeSelectsRouteAtFirstLocalizedExecute) {
    SimpleAutoDriveCommand command(&drivers, &drive, &gimbal, &odo, Mode::SPOOKY_SCRIM_2V2);
    command.initialize();
    command.execute();
    ASSERT_EQ(targets(command).size(), 1u);
    EXPECT_FLOAT_EQ(drive.getHostedTargetVelocity().magnitude(), 0);
    team(true);
    odo.relocalizeTo(4.2f, -0.35f);
    command.execute();
    ASSERT_EQ(targets(command).size(), 4u);
    EXPECT_FLOAT_EQ(targets(command)[0].position.first, 4.2f);
    EXPECT_FLOAT_EQ(targets(command)[0].position.second, -0.35f);
    EXPECT_FLOAT_EQ(targets(command)[1].position.first, 3.50f);
    EXPECT_FLOAT_EQ(targets(command)[1].position.second, -1.75f);
    EXPECT_FLOAT_EQ(targets(command)[2].position.first, 2.19f);
}

TEST_F(SimpleAutoDriveCommandTest, RouteCannotAdvanceOrDriveBeforeValidRelocalize) {
    team(false);
    SimpleAutoDriveCommand command(&drivers, &drive, &gimbal, &odo, Mode::ARCC_S2_3V3);
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
    odo.relocalizeTo(-5.25f, -3.0f);
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

TEST_F(SimpleAutoDriveCommandTest, OnlyS2DoorwayWaypointsTightenAndSquareForEitherTeam) {
    for (bool blue : {false, true}) {
        for (Mode mode : {Mode::TEST, Mode::SPOOKY_SCRIM_2V2, Mode::ARCC_S2_3V3}) checkRoute(mode, blue);
    }
}


TEST_F(SimpleAutoDriveCommandTest, CapturesFirstLocalizedNavigationAndResetsOnSchedule) {
    for (bool blue : {false, true}) {
        team(blue);
        SimpleAutoDriveCommand command(&drivers, &drive, &gimbal, &odo, Mode::TEST);
        command.initialize();
        if (!odo.isNavigationLocalized()) command.execute();
        // Deliberately distinct from both the raw pods and any historical field start.
        odo.relocalizeTo(2.3f, -0.7f);
        command.execute();
        const float sign = blue ? -1.0f : 1.0f;
        const float square[][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}};
        ASSERT_EQ(targets(command).size(), 5u);
        for (size_t i = 0; i < 5; ++i) {
            EXPECT_NEAR(targets(command)[i].position.first, 2.3f + sign * square[i][0], 1e-5f);
            EXPECT_NEAR(targets(command)[i].position.second, -0.7f + square[i][1], 1e-5f);
        }
        odo.relocalizeTo(8, 9);
        command.execute();
        EXPECT_NEAR(targets(command)[0].position.first, 2.3f, 1e-5f);
        EXPECT_NEAR(targets(command)[0].position.second, -0.7f, 1e-5f);
        command.end(true);
        command.initialize();
        command.execute();
        ASSERT_EQ(targets(command).size(), 5u);
        EXPECT_FLOAT_EQ(targets(command)[0].position.first, 8);
        EXPECT_FLOAT_EQ(targets(command)[0].position.second, 9);
        EXPECT_FLOAT_EQ(targets(command)[1].position.first, 8 + sign);
    }
}

TEST_F(SimpleAutoDriveCommandTest, RescheduleAfterDrivingKeepsBootPointAsResupply) {
    team(false);
    SimpleAutoDriveCommand command(&drivers, &drive, &gimbal, &odo, Mode::SPOOKY_SCRIM_2V2);
    command.initialize();
    odo.relocalizeTo(-3.53f, -0.426f);
    command.execute();
    hosted::sensors.podX = 1.0f;  // boot heading +y: pods (right, forward) are field (+x, +y)
    hosted::sensors.podY = 2.0f;
    odo.refresh();
    ASSERT_NEAR(odo.getX(), -2.53f, 1e-5f);
    command.end(false);  // e.g. remote reconnect or IMU recal reschedules mid-match
    command.initialize();
    command.execute();
    EXPECT_NEAR(targets(command)[0].position.first, -3.53f, 1e-5f);
    EXPECT_NEAR(targets(command)[0].position.second, -0.426f, 1e-5f);
}

TEST_F(SimpleAutoDriveCommandTest, S2RouteMirrorsForBlueAndEndsSouthOfCentralWall) {
    static_assert(static_cast<int>(Mode::SPOOKY_SCRIM_2V2) == 6);
    static_assert(static_cast<int>(Mode::ARCC_S2_3V3) == 7);
    const float red[][2] = {{-5.25f, -0.635f}, {-2.40f, -0.635f}, {-0.60f, -1.80f}};
    for (bool blue : {false, true}) {
        team(blue);
        SimpleAutoDriveCommand command(&drivers, &drive, &gimbal, &odo, Mode::ARCC_S2_3V3);
        command.initialize();
        odo.relocalizeTo(blue ? 5.4f : -5.0f, -3.3f);  // anywhere in the Starting Zone
        command.execute();
        ASSERT_EQ(targets(command).size(), 4u);
        EXPECT_FLOAT_EQ(targets(command)[0].position.first, blue ? 5.4f : -5.0f);
        EXPECT_FLOAT_EQ(targets(command)[0].position.second, -3.3f);
        for (size_t i = 0; i < 3; ++i) {
            EXPECT_FLOAT_EQ(targets(command)[i + 1].position.first, blue ? -red[i][0] : red[i][0]) << i;
            EXPECT_FLOAT_EQ(targets(command)[i + 1].position.second, red[i][1]) << i;
        }
    }
}

TEST_F(SimpleAutoDriveCommandTest, DoorwayLegsHoldQuarterTurnInsteadOfSpinning) {
    team(false);
    SimpleAutoDriveCommand command(&drivers, &drive, &gimbal, &odo, Mode::ARCC_S2_3V3);
    command.initialize();
    odo.relocalizeTo(-5.25f, -3.0f);
    command.execute();
    command.execute();  // IN_GAME at the boot point: advance to the doorway approach
    ASSERT_EQ(index(command), 1);
    EXPECT_TRUE(holding(command));
    // Booted at a quarter turn (+y) with the head centred: no turn requested.
    EXPECT_NEAR(drive.getHostedTargetVelocity().getRotation(), 0.0f, 1e-3f);
    setIndex(command, 3);
    command.execute();
    EXPECT_FALSE(holding(command));
    EXPECT_LT(drive.getHostedTargetVelocity().getRotation(), -1.0f);
}

TEST_F(SimpleAutoDriveCommandTest, OutboundAppliesAndRestoresTolerance) {
    checkTraversal(false);
}

TEST_F(SimpleAutoDriveCommandTest, ReturnAppliesAndRestoresTolerance) {
    checkTraversal(true);
}
}  // namespace commands
