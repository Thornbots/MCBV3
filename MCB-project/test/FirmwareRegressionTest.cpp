// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include <iterator>
#include <limits>
#include <type_traits>
#include <vector>
#include "communication/UARTCommunication.hpp"
#include "hosted/clock.hpp"
#include "tap/control/command_mapping.hpp"
#include "tap/control/command.hpp"
#include "modm/container/deque.hpp"
#include "subsystems/odometry/OdometrySubsystem.hpp"
#include "subsystems/odometry/OdometrySubsystemConstants.hpp"

namespace {
class Drivers : public tap::Drivers {
public:
    Drivers() = default;
};
class Command : public tap::control::Command {
public:
    const char* getName() const override { return "test"; }
    void initialize() override {}
    void execute() override {}
    void end(bool) override {}
    bool isFinished() const override { return false; }
};
class Mapping : public tap::control::CommandMapping {
public:
    explicit Mapping(const std::vector<tap::control::Command*>& commands)
        : CommandMapping(nullptr, commands, {}) {}
    void executeCommandMapping(const tap::control::RemoteMapState&) override {}
};
}

TEST(OdometryFramesTest, RelocalizationChangesNavigationButPreservesRawAim) {
    src::Drivers drivers;
    hosted::sensors = {};
    subsystems::OdometrySubsystem odometry(&drivers, nullptr);
    const float rawX = odometry.getAimX();
    const float rawY = odometry.getAimY();
    EXPECT_FLOAT_EQ(rawX, -odo::START_X);
    EXPECT_FLOAT_EQ(rawY, odo::START_Y);
    odometry.relocalizeTo(1.25f, -0.75f);
    EXPECT_FLOAT_EQ(odometry.getX(), 1.25f);
    EXPECT_FLOAT_EQ(odometry.getY(), -0.75f);
    EXPECT_FLOAT_EQ(odometry.getAimX(), rawX);
    EXPECT_FLOAT_EQ(odometry.getAimY(), rawY);
    hosted::sensors.podX = 0.3f;
    hosted::sensors.podY = 0.5f;
    EXPECT_NEAR(odometry.getAimX(), rawX + 0.3f, 1e-6f);
    EXPECT_NEAR(odometry.getAimY(), rawY + 0.5f, 1e-6f);
    EXPECT_NEAR(odometry.getX(), 1.55f, 1e-6f);
    EXPECT_NEAR(odometry.getY(), -0.25f, 1e-6f);
    odometry.relocalizeTo(-2.0f, 3.0f);
    EXPECT_FLOAT_EQ(odometry.getX(), -2.0f);
    EXPECT_FLOAT_EQ(odometry.getY(), 3.0f);
    EXPECT_NEAR(odometry.getAimX(), rawX + 0.3f, 1e-6f);
    EXPECT_NEAR(odometry.getAimY(), rawY + 0.5f, 1e-6f);
    hosted::sensors = {};
}

TEST(CommandMappingTest, RemovesNullCommandsAndPreservesOrder) {
    Command first, second;
    Mapping mapping({nullptr, &first, nullptr, &second, nullptr});
    EXPECT_EQ(mapping.getAssociatedCommands(),
              (std::vector<tap::control::Command*>{&first, &second}));
    Mapping empty({nullptr, nullptr});
    EXPECT_TRUE(empty.getAssociatedCommands().empty());
}

TEST(UARTCommunicationTest, RejectsEmptyAndOversizedMessages) {
    hosted::timeMs = 0;
    Drivers drivers;
    communication::UARTCommunication uart(
        &drivers, tap::communication::serial::Uart::Uart1, true);
    communication::UARTCommunication::ReceivedSerialMessage message{};
    message.header.dataLength = 0;
    uart.messageReceiveCallback(message);
    EXPECT_FALSE(uart.hasNewMessage());
    message.header.dataLength = sizeof(message.data) + 1;
    uart.messageReceiveCallback(message);
    EXPECT_FALSE(uart.hasNewMessage());
    message.header.dataLength = sizeof(message.data);
    message.messageType = 42;
    message.data[sizeof(message.data) - 1] = 99;
    uart.messageReceiveCallback(message);
    ASSERT_TRUE(uart.hasNewMessage());
    auto received = uart.getLastMsg();
    EXPECT_EQ(received.messageType, 42);
    EXPECT_EQ(received.dataLength, sizeof(message.data));
    EXPECT_EQ(received.data[sizeof(message.data) - 1], 99);
}

TEST(UARTCommunicationTest, TimeoutSurvivesMillisecondClockRollover) {
    hosted::timeMs = std::numeric_limits<uint32_t>::max() - 500;
    Drivers drivers;
    communication::UARTCommunication uart(
        &drivers, tap::communication::serial::Uart::Uart1, true);
    EXPECT_EQ(uart.getCurrentTime(), hosted::timeMs);
    EXPECT_TRUE(uart.isConnected());
    hosted::timeMs += 1000;
    EXPECT_TRUE(uart.isConnected());
    ++hosted::timeMs;
    EXPECT_FALSE(uart.isConnected());
}

TEST(UARTCommunicationTest, MappingPingsSurviveLatestTelemetryAndFailedTransmit) {
    hosted::timeMs = 0;
    src::Drivers drivers;
    communication::UARTCommunication::ReceivedSerialMessage message{};
    message.messageType = 5;
    message.header.dataLength = 1;
    message.data[0] = 2;
    drivers.uart.messageReceiveCallback(message);
    message.data[0] = 4;
    drivers.uart.messageReceiveCallback(message);
    for (uint16_t type : {2, 3}) {
        message.messageType = type;
        message.header.dataLength = 11;
        drivers.uart.messageReceiveCallback(message);
    }
    EXPECT_EQ(drivers.uart.getLastMsg().messageType, 3);
    drivers.mapping.tick(0, [](uint8_t) { return false; });
    EXPECT_TRUE(drivers.mapping.hasPendingPing());
    std::vector<uint8_t> sent;
    for (int i = 0; i < 3; ++i)
        drivers.mapping.tick(1, [&](uint8_t n) { sent.push_back(n); return true; });
    EXPECT_EQ(sent, (std::vector<uint8_t>{2, 4}));
    EXPECT_FALSE(drivers.mapping.hasPendingPing());
    EXPECT_EQ(drivers.mapping.getState(), src::MappingMachine::State::Main);
}

TEST(ModmIteratorTest, SupportsStandardAlgorithmsWithConstTraits) {
    using Iterator = modm::BoundedDeque<int, 4>::const_iterator;
    static_assert(std::is_same_v<std::iterator_traits<Iterator>::pointer, const int*>);
    static_assert(std::is_same_v<std::iterator_traits<Iterator>::reference, const int&>);
    modm::BoundedDeque<int, 4> values;
    values.append(3);
    values.append(7);
    EXPECT_EQ(std::distance(values.begin(), values.end()), 2);
    EXPECT_EQ(*std::next(values.begin()), 7);
}
