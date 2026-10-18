// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include "hosted/clock.hpp"
#include "hosted/uart.hpp"
#include "robots/sentry/SentryHardware.hpp"
#include "subsystems/drivetrain/DrivetrainStopCommand.hpp"

namespace {
class ParkedChassisTest : public ::testing::Test {
protected:
    src::Drivers drivers;
    robots::SentryHardware hardware{&drivers};
    subsystems::DrivetrainSubsystem drive{&drivers, &hardware.driveMotor1,
        &hardware.driveMotor2, &hardware.driveMotor3, &hardware.driveMotor4};
    subsystems::GimbalSubsystem gimbal{&drivers, &hardware.yawMotor, &hardware.pitchMotor};
    subsystems::OdometrySubsystem odometry{&drivers, &hardware.odoMotor};
    commands::DrivetrainStopCommand command{&drivers, &drive, &gimbal, &odometry};

    void SetUp() override {
        hosted::timeMs = 0;
        hosted::sensors = {};
        drivers.remote.initialize();
        tap::communication::serial::RefSerial::ReceivedSerialMessage game{};
        game.messageType = tap::communication::serial::RefSerial::REF_MESSAGE_TYPE_GAME_STATUS;
        game.header.dataLength = 11;
        game.data[0] = static_cast<uint8_t>(RefSerialData::Rx::GameStage::IN_GAME) << 4;
        drivers.refSerial.messageReceiveCallback(game);
        tap::communication::serial::RefSerial::ReceivedSerialMessage robot{};
        robot.messageType = tap::communication::serial::RefSerial::REF_MESSAGE_TYPE_ROBOT_STATUS;
        robot.header.dataLength = 13;
        robot.data[10] = 100;
        robot.data[12] = static_cast<uint8_t>(RefSerialData::Rx::RobotPower::CHASSIS_HAS_POWER);
        drivers.refSerial.messageReceiveCallback(robot);
        drive.imuAngle = 0;
        odometry.refresh();
        command.initialize();
    }

    void TearDown() override { hosted::sensors = {}; }

    void connectController(bool afterTimeout = false) {
        const uint8_t remote[18] = {0, 4, 32, 0, 1, 88, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4};
        const auto queue = afterTimeout ? hosted::queueUartAfterDiscard : hosted::queueUart;
        queue(tap::communication::serial::Uart::Uart3, remote, sizeof(remote));
        drivers.remote.read();
        ASSERT_TRUE(drivers.remote.isConnected());
    }

    void disconnectController() {
        hosted::timeMs += 101;
        drivers.remote.read();
        ASSERT_FALSE(drivers.remote.isConnected());
    }

    void expectZeroOutput() {
        for (const auto* motor : {&hardware.driveMotor1, &hardware.driveMotor2,
                &hardware.driveMotor3, &hardware.driveMotor4}) {
            EXPECT_EQ(motor->getOutputDesired(), 0);
        }
    }

    void establishActiveHold() {
        connectController();
        command.execute();
        hosted::sensors.podX = 0.1f;
        command.execute();
        ASSERT_GT(drive.getHostedTargetVelocity().magnitude(), 0.0f);
        ASSERT_TRUE(hardware.driveMotor1.getOutputDesired() != 0 ||
            hardware.driveMotor2.getOutputDesired() != 0 ||
            hardware.driveMotor3.getOutputDesired() != 0 ||
            hardware.driveMotor4.getOutputDesired() != 0);
    }
};

TEST_F(ParkedChassisTest, ControllerAbsentClearsExistingCurrentDespiteInGamePower) {
    ASSERT_FALSE(drivers.remote.isConnected());
    for (auto* motor : {&hardware.driveMotor1, &hardware.driveMotor2,
            &hardware.driveMotor3, &hardware.driveMotor4}) motor->setDesiredOutput(1000);
    command.execute();
    expectZeroOutput();
}

TEST_F(ParkedChassisTest, ControllerLossZerosActiveHoldAndStaysZeroWhileDisplaced) {
    establishActiveHold();
    disconnectController();
    command.execute();
    expectZeroOutput();
    hosted::sensors.podX = 0.2f;
    for (int tick = 0; tick < 10; ++tick) {
        command.execute();
        expectZeroOutput();
    }
}

TEST_F(ParkedChassisTest, ReconnectHoldsCurrentPositionInsteadOfReturningToOldPark) {
    establishActiveHold();
    disconnectController();
    command.execute();
    hosted::sensors.podX = 0.2f;
    connectController(true);
    command.execute();
    EXPECT_FLOAT_EQ(drive.getHostedTargetVelocity().magnitude(), 0.0f);
    hosted::sensors.podX = 0.21f;
    command.execute();
    EXPECT_GT(drive.getHostedTargetVelocity().magnitude(), 0.0f);
}
}  // namespace
