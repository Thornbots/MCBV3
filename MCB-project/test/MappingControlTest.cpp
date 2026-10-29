// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include <sys/socket.h>
#include <unistd.h>
#include "hosted/clock.hpp"
#include "hosted/uart.hpp"
#include "robots/RobotControl.hpp"

namespace {
class MappingControlTest : public ::testing::Test {
protected:
    src::Drivers drivers;
    robots::SentryControl control{&drivers};
    int sockets[2] = {-1, -1};

    void SetUp() override {
        hosted::timeMs = 0;
        hosted::sensors = {};
        ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0, sockets), 0);
        hosted::setJetsonFd(sockets[0]);
        drivers.remote.initialize();
        connect();
        control.initialize();
    }
    void TearDown() override {
        close(sockets[0]);
        close(sockets[1]);
        hosted::setJetsonFd(-1);
        hosted::sensors = {};
    }
    void connect(bool afterTimeout = false) {
        // Both left axes and right horizontal at maximum; switches choose normal triggers.
        const uint16_t channels[] = {1684, 1024, 1684, 1684};
        uint64_t bits = 0;
        for (int i = 0; i < 4; ++i) bits |= static_cast<uint64_t>(channels[i]) << (11 * i);
        bits |= uint64_t{0xD} << 44;
        uint8_t remote[18]{};
        for (int i = 0; i < 6; ++i) remote[i] = bits >> (8 * i);
        remote[17] = 4;
        const auto queue = afterTimeout ? hosted::queueUartAfterDiscard : hosted::queueUart;
        queue(tap::communication::serial::Uart::Uart3, remote, sizeof(remote));
        drivers.remote.read();
        ASSERT_TRUE(drivers.remote.isConnected());
    }
    void tick() {
        control.updateMapping();
        drivers.commandScheduler.run();
        control.update();
        ++hosted::timeMs;
    }
    void expectRestricted() {
        EXPECT_TRUE(drivers.commandScheduler.isCommandScheduled(&control.mappingCommand));
        EXPECT_FALSE(drivers.commandScheduler.isCommandScheduled(&control.autoFire));
        EXPECT_FALSE(drivers.commandScheduler.isCommandScheduled(&control.noSpinDriveCommand));
        EXPECT_FALSE(drivers.commandScheduler.isCommandScheduled(&control.beybladeJoystick));
        EXPECT_EQ(control.hardware.indexMotor.getOutputDesired(), 0);
        EXPECT_EQ(control.hardware.flywheelMotor1.getOutputDesired(), 0);
        EXPECT_EQ(control.hardware.flywheelMotor2.getOutputDesired(), 0);
        EXPECT_FLOAT_EQ(control.gimbal.getPrevTargetPitch(), 0);
    }
};
}

TEST_F(MappingControlTest, PersistentRestrictionsSurviveTriggersAndStateChanges) {
    drivers.commandScheduler.addCommand(&control.autoFire);
    drivers.commandScheduler.addCommand(&control.beybladeJoystick);
    drivers.mapping.requestStart();
    tick();
    expectRestricted();
    EXPECT_FLOAT_EQ(control.drivetrain.getHostedTargetVelocity().magnitude(), 0);
    drivers.mapping.receivePing(2, hosted::timeMs);
    for (int i = 0; i < 5; ++i) { tick(); expectRestricted(); }
    auto drive = control.drivetrain.getHostedTargetVelocity();
    EXPECT_NEAR(drive.magnitude(), 0.4f, 1e-6f);
    EXPECT_LE(std::abs(drive.getRotation()), 0.5f);
    drivers.mapping.receivePing(4, hosted::timeMs);
    tick();
    expectRestricted();
    drivers.mapping.requestSave();
    tick();
    EXPECT_EQ(drivers.mapping.getState(), src::MappingMachine::State::Mapping);
    expectRestricted();
    drivers.mapping.receivePing(3, hosted::timeMs);
    tick();
    hosted::timeMs += src::MappingMachine::RETRY_MS;
    connect(true);
    tick();
    drivers.mapping.receivePing(2, hosted::timeMs);
    drivers.mapping.requestSave();
    tick();
    EXPECT_EQ(drivers.mapping.getState(), src::MappingMachine::State::Done);
    expectRestricted();
    EXPECT_FLOAT_EQ(control.drivetrain.getHostedTargetVelocity().magnitude(), 0);
}

TEST_F(MappingControlTest, DisconnectStopsDriveAndReconnectRestoresRestrictions) {
    drivers.mapping.requestStart();
    drivers.mapping.receivePing(2, 0);
    tick();
    EXPECT_GT(control.drivetrain.getHostedTargetVelocity().magnitude(), 0);
    hosted::timeMs += 101;
    drivers.remote.read();
    ASSERT_FALSE(drivers.remote.isConnected());
    tick();
    for (const auto* motor : {&control.hardware.driveMotor1, &control.hardware.driveMotor2,
            &control.hardware.driveMotor3, &control.hardware.driveMotor4})
        EXPECT_EQ(motor->getOutputDesired(), 0);
    EXPECT_EQ(drivers.mapping.getState(), src::MappingMachine::State::Ready);
    connect(true);
    tick();
    expectRestricted();
}

TEST_F(MappingControlTest, PendingControlPingHasPriorityOverTelemetryAndRetriesBusyUart) {
    uint8_t fill[4096]{};
    while (write(sockets[0], fill, sizeof(fill)) > 0) {}
    drivers.mapping.receivePing(2, 0);
    control.updateMapping();
    EXPECT_TRUE(drivers.mapping.hasPendingPing());
    hosted::timeMs = 10;
    control.jetson.refresh();
    EXPECT_TRUE(drivers.mapping.hasPendingPing());
    uint8_t buffer[65536];
    while (read(sockets[1], buffer, sizeof(buffer)) > 0) {}
    control.updateMapping();
    ASSERT_FALSE(drivers.mapping.hasPendingPing());
    ASSERT_EQ(read(sockets[1], buffer, sizeof(buffer)), 10);
    EXPECT_EQ(buffer[5], subsystems::PING);
    EXPECT_EQ(buffer[7], 2);
}

TEST_F(MappingControlTest, ExistingStoppedControlHasPrecedenceOverMappingRestrictions) {
    drivers.mapping.requestStart();
    drivers.mapping.receivePing(2, 0);
    tick();
    ASSERT_GT(control.drivetrain.getHostedTargetVelocity().magnitude(), 0);
    control.stopForImuRecal();
    tick();
    EXPECT_FALSE(drivers.commandScheduler.isCommandScheduled(&control.mappingCommand));
    EXPECT_FLOAT_EQ(control.drivetrain.getHostedTargetVelocity().magnitude(), 0);
}

TEST_F(MappingControlTest, ForwardHeadTracksChassisHeadingRatherThanWorldZero) {
    drivers.mapping.requestStart();
    for (float yaw : {0.8f, 1.7f, -0.5f}) {
        hosted::sensors.yaw = yaw;
        control.gimbal.refresh();
        tick();
        expectRestricted();
        const float chassisYaw = drivers.bmi088.getYaw() - control.gimbal.getYawEncoderValue();
        EXPECT_NEAR(std::remainder(control.gimbal.getHostedTargetYaw() - chassisYaw, 2 * PI), 0, 1e-5f);
        EXPECT_FLOAT_EQ(control.gimbal.getPrevTargetPitch(), 0);
    }
}

TEST_F(MappingControlTest, StartWhileDisconnectedClearsOutputsLeftByMain) {
    hosted::timeMs += 101;
    drivers.remote.read();
    ASSERT_FALSE(drivers.remote.isConnected());
    for (auto* motor : {&control.hardware.driveMotor1, &control.hardware.driveMotor2,
            &control.hardware.driveMotor3, &control.hardware.driveMotor4,
            &control.hardware.flywheelMotor1, &control.hardware.flywheelMotor2,
            &control.hardware.indexMotor}) motor->setDesiredOutput(1000);
    drivers.mapping.requestStart();
    tick();
    for (const auto* motor : {&control.hardware.driveMotor1, &control.hardware.driveMotor2,
            &control.hardware.driveMotor3, &control.hardware.driveMotor4,
            &control.hardware.flywheelMotor1, &control.hardware.flywheelMotor2,
            &control.hardware.indexMotor}) EXPECT_EQ(motor->getOutputDesired(), 0);
}
