// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include <iterator>
#include <limits>
#include <type_traits>
#include "communication/UARTCommunication.hpp"
#include "hosted/clock.hpp"
#include "tap/control/command_mapping.hpp"
#include "tap/control/command.hpp"
#include "modm/container/deque.hpp"

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
