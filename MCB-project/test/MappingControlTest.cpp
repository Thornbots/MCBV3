// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include <sys/socket.h>
#include <unistd.h>
#include "hosted/clock.hpp"
#include "hosted/uart.hpp"
#include "robots/RobotControl.hpp"

// Observe actual GPIO calls; Taproot's hosted GPIO implementation is a no-op.
namespace {
bool ledStates[3]{};
float buzzerDuty = 0;
}
extern "C" void __wrap__ZN3tap4gpio4Leds3setENS1_6LedPinEb(
    tap::gpio::Leds*, tap::gpio::Leds::LedPin pin, bool value) {
    ledStates[pin] = value;
}
extern "C" void __wrap__ZN3tap6buzzer8playNoteEPNS_4gpio3PwmEj(
    tap::gpio::Pwm*, uint32_t frequency) {
    EXPECT_EQ(frequency, 1000U);
    buzzerDuty = 0.5f;
}
extern "C" void __wrap__ZN3tap6buzzer13silenceBuzzerEPNS_4gpio3PwmE(tap::gpio::Pwm*) {
    buzzerDuty = 0;
}

namespace {
class MappingControlTest : public ::testing::Test {
protected:
    src::Drivers drivers;
    robots::SentryControl control{&drivers};
    int sockets[2] = {-1, -1};

    void SetUp() override {
        hosted::timeMs = 0;
        hosted::sensors = {};
        for (auto& led : ledStates) led = false;
        buzzerDuty = 0;
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
    void connect(bool afterTimeout = false, uint16_t rightVertical = 1024) {
        // Both left axes and right horizontal at maximum; switches choose normal triggers.
        const uint16_t channels[] = {1684, rightVertical, 1684, 1684};
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
    // Feeds the remote and drains Jetson output every loop of a long hold.
    void holdRightStick(bool up, uint32_t ms) {
        uint8_t buffer[4096];
        for (uint32_t i = 0; i < ms; ++i) {
            connect(false, up ? 1684 : 1024);
            tick();
            while (read(sockets[1], buffer, sizeof(buffer)) > 0) {}
        }
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
    hosted::sensors.yaw = 1.7f;
    control.gimbal.refresh();
    drivers.mapping.receivePing(4, hosted::timeMs);
    tick();
    EXPECT_EQ(drivers.mapping.getState(), src::MappingMachine::State::Mapping);
    expectRestricted();
    drive = control.drivetrain.getHostedTargetVelocity();
    EXPECT_NEAR(drive.magnitude(), 0.4f, 1e-6f);
    EXPECT_LE(std::abs(drive.getRotation()), 0.5f);
    const float chassisYaw = drivers.bmi088.getYaw() - control.gimbal.getYawEncoderValue();
    EXPECT_NEAR(std::remainder(control.gimbal.getHostedTargetYaw() - chassisYaw, 2 * PI), 0, 1e-5f);
}

TEST_F(MappingControlTest, SetupFailKeepsDoneStoppedAndHeadForward) {
    drivers.mapping.requestStart();
    tick();
    drivers.mapping.receivePing(6, hosted::timeMs);
    tick();
    EXPECT_EQ(drivers.mapping.getState(), src::MappingMachine::State::Done);
    expectRestricted();
    EXPECT_FLOAT_EQ(control.drivetrain.getHostedTargetVelocity().magnitude(), 0);
    const float chassisYaw = drivers.bmi088.getYaw() - control.gimbal.getYawEncoderValue();
    EXPECT_NEAR(std::remainder(control.gimbal.getHostedTargetYaw() - chassisYaw, 2 * PI), 0, 1e-5f);
}

TEST_F(MappingControlTest, DisconnectStopsEveryMotorAndReconnectRestoresRestrictions) {
    drivers.mapping.requestStart();
    drivers.mapping.receivePing(2, 0);
    tick();
    EXPECT_GT(control.drivetrain.getHostedTargetVelocity().magnitude(), 0);
    EXPECT_NE(control.hardware.odoMotor.getOutputDesired(), 0);
    hosted::timeMs += 101;
    drivers.remote.read();
    ASSERT_FALSE(drivers.remote.isConnected());
    tick();
    tick();
    EXPECT_FALSE(drivers.commandScheduler.isCommandScheduled(&control.mappingCommand));
    EXPECT_TRUE(drivers.commandScheduler.isCommandScheduled(&control.odoStop));
    EXPECT_TRUE(drivers.commandScheduler.isCommandScheduled(&control.stopGimbal));
    for (const auto* motor : {&control.hardware.driveMotor1, &control.hardware.driveMotor2,
            &control.hardware.driveMotor3, &control.hardware.driveMotor4,
            &control.hardware.yawMotor, &control.hardware.pitchMotor, &control.hardware.odoMotor,
            &control.hardware.flywheelMotor1, &control.hardware.flywheelMotor2,
            &control.hardware.indexMotor})
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
    control.jetson.refresh();
    EXPECT_TRUE(drivers.mapping.hasPendingPing());
    hosted::timeMs = 10;
    control.jetson.refresh();
    EXPECT_TRUE(drivers.mapping.hasPendingPing());
    uint8_t buffer[65536];
    while (read(sockets[1], buffer, sizeof(buffer)) > 0) {}
    hosted::timeMs = 10000;
    control.jetson.refresh();
    ASSERT_FALSE(drivers.mapping.hasPendingPing());
    ASSERT_GE(read(sockets[1], buffer, sizeof(buffer)), 20);
    EXPECT_EQ(buffer[5], subsystems::PING);
    EXPECT_EQ(buffer[7], 2);
    EXPECT_TRUE(buffer[15] == subsystems::POSE || buffer[15] == subsystems::REF_SYS);
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

TEST_F(MappingControlTest, StartWhileDisconnectedKeepsEveryMotorStopped) {
    hosted::timeMs += 101;
    drivers.remote.read();
    ASSERT_FALSE(drivers.remote.isConnected());
    tick();  // main schedules its stop defaults once the remote is gone
    for (auto* motor : {&control.hardware.driveMotor1, &control.hardware.driveMotor2,
            &control.hardware.driveMotor3, &control.hardware.driveMotor4,
            &control.hardware.yawMotor, &control.hardware.pitchMotor, &control.hardware.odoMotor,
            &control.hardware.flywheelMotor1, &control.hardware.flywheelMotor2,
            &control.hardware.indexMotor}) motor->setDesiredOutput(1000);
    drivers.mapping.requestStart();
    tick();
    EXPECT_FALSE(drivers.commandScheduler.isCommandScheduled(&control.mappingCommand));
    for (const auto* motor : {&control.hardware.driveMotor1, &control.hardware.driveMotor2,
            &control.hardware.driveMotor3, &control.hardware.driveMotor4,
            &control.hardware.yawMotor, &control.hardware.pitchMotor, &control.hardware.odoMotor,
            &control.hardware.flywheelMotor1, &control.hardware.flywheelMotor2,
            &control.hardware.indexMotor}) EXPECT_EQ(motor->getOutputDesired(), 0);
}

TEST_F(MappingControlTest, PodsPointInEveryMappingStateAndStopWithoutRemote) {
    using State = src::MappingMachine::State;
    for (State state : {State::Setup, State::Ready, State::Mapping, State::Done}) {
        SCOPED_TRACE(static_cast<int>(state));
        if (state != State::Setup) connect(true);  // the previous state ended disconnected
        drivers.mapping = src::MappingMachine{};
        drivers.mapping.requestStart();
        if (state == State::Ready || state == State::Mapping)
            drivers.mapping.receivePing(2, hosted::timeMs);
        if (state == State::Mapping) drivers.mapping.receivePing(4, hosted::timeMs);
        if (state == State::Done) drivers.mapping.receivePing(6, hosted::timeMs);
        control.hardware.odoMotor.setDesiredOutput(0);
        tick();
        EXPECT_EQ(drivers.mapping.getState(), state);
        expectRestricted();
        EXPECT_NE(control.hardware.odoMotor.getOutputDesired(), 0);
        EXPECT_EQ(ledStates[tap::gpio::Leds::Green], state == State::Ready);
        EXPECT_FALSE(ledStates[tap::gpio::Leds::Red]);
        EXPECT_FALSE(ledStates[tap::gpio::Leds::Blue]);
        EXPECT_FALSE(drivers.commandScheduler.isCommandScheduled(&control.odoStop));
        EXPECT_FALSE(drivers.commandScheduler.isCommandScheduled(&control.odoPointForwards));

        hosted::timeMs += 101;
        drivers.remote.read();
        ASSERT_FALSE(drivers.remote.isConnected());
        tick();
        tick();
        EXPECT_EQ(drivers.mapping.getState(), state);
        EXPECT_FALSE(drivers.commandScheduler.isCommandScheduled(&control.mappingCommand));
        EXPECT_EQ(control.hardware.odoMotor.getOutputDesired(), 0);
        EXPECT_EQ(control.hardware.yawMotor.getOutputDesired(), 0);
        EXPECT_EQ(control.hardware.pitchMotor.getOutputDesired(), 0);
    }
}

TEST_F(MappingControlTest, CommandOwnsReadyIndicatorsAndEndClearsThem) {
    drivers.mapping.requestStart();
    tick();
    EXPECT_FALSE(ledStates[tap::gpio::Leds::Green]);
    EXPECT_FLOAT_EQ(buzzerDuty, 0);
    drivers.mapping.receivePing(2, hosted::timeMs);
    const auto readySince = hosted::timeMs;
    for (uint32_t elapsed : {0u, 99u, 100u, 999u, 1000u, 1100u}) {
        hosted::timeMs = readySince + elapsed;
        drivers.remote.read();
        connect(!drivers.remote.isConnected());
        ledStates[tap::gpio::Leds::Red] = true;
        ledStates[tap::gpio::Leds::Blue] = true;
        tick();
        EXPECT_TRUE(ledStates[tap::gpio::Leds::Green]);
        EXPECT_FALSE(ledStates[tap::gpio::Leds::Red]);
        EXPECT_FALSE(ledStates[tap::gpio::Leds::Blue]);
        EXPECT_FLOAT_EQ(buzzerDuty, elapsed % 1000 < 100 ? 0.5f : 0.0f);
    }
    hosted::timeMs = readySince + 2000;
    control.mappingCommand.execute();
    EXPECT_FLOAT_EQ(buzzerDuty, 0.5f);
    control.mappingCommand.end(true);
    EXPECT_FALSE(ledStates[tap::gpio::Leds::Green]);
    EXPECT_FLOAT_EQ(buzzerDuty, 0);
    drivers.mapping.receivePing(4, hosted::timeMs);
    tick();
    EXPECT_FALSE(ledStates[tap::gpio::Leds::Green]);
    EXPECT_FLOAT_EQ(buzzerDuty, 0);
    drivers.mapping.receivePing(2, hosted::timeMs);
    drivers.mapping.requestSave();
    tick();
    EXPECT_EQ(drivers.mapping.getState(), src::MappingMachine::State::Done);
    EXPECT_FALSE(ledStates[tap::gpio::Leds::Green]);
    EXPECT_FLOAT_EQ(buzzerDuty, 0);
}

TEST_F(MappingControlTest, JetsonServicesPingInMainAndEveryMappingState) {
    using State = src::MappingMachine::State;
    uint8_t buffer[4096];
    const auto expectPing = [&](uint8_t number) {
        control.jetson.refresh();
        const auto count = read(sockets[1], buffer, sizeof(buffer));
        ASSERT_GE(count, 10);
        EXPECT_EQ(buffer[5], subsystems::PING);
        EXPECT_EQ(buffer[7], number);
    };
    for (State state : {State::Main, State::Setup, State::Ready, State::Mapping, State::Done}) {
        SCOPED_TRACE(static_cast<int>(state));
        drivers.mapping = src::MappingMachine{};
        if (state != State::Main) {
            drivers.mapping.requestStart();
            expectPing(1);
            drivers.mapping.receivePing(1, hosted::timeMs);
            expectPing(1);
        } else {
            drivers.mapping.receivePing(0, hosted::timeMs);
            expectPing(0);
        }
        if (state == State::Ready || state == State::Mapping) {
            drivers.mapping.receivePing(2, hosted::timeMs);
            expectPing(2);
        }
        if (state == State::Mapping) {
            drivers.mapping.receivePing(4, hosted::timeMs);
            expectPing(4);
        }
        if (state == State::Done) {
            drivers.mapping.receivePing(6, hosted::timeMs);
            expectPing(6);
        }
        ASSERT_EQ(drivers.mapping.getState(), state);
        for (uint8_t number : {2, 4, 6}) {
            drivers.mapping.receivePing(number, hosted::timeMs);
            expectPing(number);
        }
    }
    drivers.mapping = src::MappingMachine{};
    drivers.mapping.requestStart();
    expectPing(1);
    drivers.mapping.receivePing(2, hosted::timeMs);
    expectPing(2);
    drivers.mapping.requestSave();
    expectPing(3);
    drivers.mapping.receivePing(3, hosted::timeMs);
    expectPing(3);
}

TEST_F(MappingControlTest, CalibrationFallbackServicesMainAndStartWithoutScheduler) {
    drivers.recal.setIsFirstCalibrating();
    ASSERT_FALSE(drivers.recal.getIsImuReady());
    uint8_t buffer[64];
    drivers.mapping.receivePing(0, hosted::timeMs);
    control.updateMapping();
    ASSERT_EQ(read(sockets[1], buffer, sizeof(buffer)), 10);
    EXPECT_EQ(buffer[7], 0);
    drivers.mapping.requestStart();
    control.updateMapping();
    ASSERT_EQ(read(sockets[1], buffer, sizeof(buffer)), 10);
    EXPECT_EQ(buffer[7], 1);
}

TEST_F(MappingControlTest, RightStickHoldStartsThenSavesOncePerHold) {
    using State = src::MappingMachine::State;
    const auto holdMs = static_cast<uint32_t>(MAPPING_HOLD_TIME);
    tick();
    drivers.commandScheduler.addCommand(&control.lookJoystickOffset);
    holdRightStick(true, holdMs - 10);
    EXPECT_EQ(drivers.mapping.getState(), State::Main);
    holdRightStick(true, 20);
    EXPECT_EQ(drivers.mapping.getState(), State::Setup);
    holdRightStick(true, 10);
    expectRestricted();
    drivers.mapping.receivePing(1, hosted::timeMs);  // ACK the sent start.

    // The start hold carries into Ready without saving until released.
    drivers.mapping.receivePing(2, hosted::timeMs);
    holdRightStick(true, 2 * holdMs);
    EXPECT_EQ(drivers.mapping.getState(), State::Ready);
    holdRightStick(false, 1);
    holdRightStick(true, holdMs + 10);
    EXPECT_EQ(drivers.mapping.getState(), State::Done);

    // A failed final save resumes Mapping; another hold sends an incomplete save.
    drivers.mapping.receivePing(4, hosted::timeMs);
    holdRightStick(true, holdMs + 10);
    holdRightStick(false, 1);
    EXPECT_EQ(drivers.mapping.getState(), State::Mapping);
    drivers.mapping.receivePing(3, hosted::timeMs);  // No save awaits this ACK.
    EXPECT_FALSE(drivers.mapping.hasPendingPing());
    holdRightStick(true, holdMs + 10);
    drivers.mapping.receivePing(3, hosted::timeMs);  // ACK queues the final leg.
    EXPECT_TRUE(drivers.mapping.hasPendingPing());
    EXPECT_EQ(drivers.mapping.getState(), State::Mapping);
    expectRestricted();
}
