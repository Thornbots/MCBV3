// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include <algorithm>
#include <limits>
#include <vector>
#include "MappingMachine.hpp"
#include "robots/sentry/MappingDrive.hpp"

namespace {
using Machine = src::MappingMachine;
using State = Machine::State;

Machine inState(State state) {
    Machine machine;
    if (state == State::Main) return machine;
    machine.requestStart();
    if (state == State::Setup) return machine;
    machine.receivePing(2, 0);
    if (state == State::Ready) return machine;
    if (state == State::Done) machine.requestSave();
    else machine.receivePing(4, 0);
    return machine;
}
constexpr State states[] = {State::Main, State::Setup, State::Ready, State::Mapping, State::Done};
}

TEST(MappingMachineTest, EveryStateAndInputTransition) {
    for (auto state : states) {
        for (uint8_t input : {0, 1, 2, 3, 4, 5, 255}) {
            auto machine = inState(state);
            machine.receivePing(input, 100);
            auto expected = state;
            if (input == 2 && (state == State::Setup || state == State::Mapping)) expected = State::Ready;
            if (input == 4 && state == State::Ready) expected = State::Mapping;
            EXPECT_EQ(machine.getState(), expected) << static_cast<int>(state) << ":" << +input;
        }
        auto start = inState(state);
        start.requestStart();
        EXPECT_EQ(start.getState(), state == State::Main ? State::Setup : state);
        auto save = inState(state);
        save.requestSave();
        EXPECT_EQ(save.getState(), state == State::Ready ? State::Done : state);
        EXPECT_EQ(save.canDrive(), save.getState() == State::Ready || save.getState() == State::Mapping);
        EXPECT_EQ(save.isActive(), save.getState() != State::Main);
    }
}

TEST(MappingMachineTest, ZeroOnlyEchoesInMainAndUnknownIsIgnored) {
    for (auto state : states) {
        auto machine = inState(state);
        std::vector<uint8_t> sent;
        auto send = [&](uint8_t n) { sent.push_back(n); return true; };
        for (int i = 0; i < 10; ++i) machine.tick(0, send);
        sent.clear();
        machine.receivePing(0, 1);
        machine.receivePing(255, 1);
        machine.tick(1, send);
        EXPECT_EQ(sent, state == State::Main ? std::vector<uint8_t>{0} : std::vector<uint8_t>{});
    }
}

TEST(MappingMachineTest, EvenPingsEchoEveryTimeAndDuplicatesDoNotRestartReadyCadence) {
    Machine machine;
    machine.requestStart();
    machine.receivePing(2, 10);
    machine.receivePing(2, 500);
    EXPECT_FALSE(machine.readyBeep(500));
    EXPECT_TRUE(machine.readyBeep(1010));
    machine.receivePing(4, 1100);
    machine.receivePing(4, 1200);
    EXPECT_EQ(machine.getState(), State::Mapping);
    std::vector<uint8_t> sent;
    for (int i = 0; i < 4; ++i)
        machine.tick(1200, [&](uint8_t n) { sent.push_back(n); return true; });
    EXPECT_EQ(sent, (std::vector<uint8_t>{2, 2, 4, 4}));
}

TEST(MappingMachineTest, StartCancelsUnsentMainOnlyLinkEcho) {
    Machine machine;
    machine.receivePing(0, 0);
    machine.requestStart();
    std::vector<uint8_t> sent;
    for (int i = 0; i < 3; ++i)
        machine.tick(0, [&](uint8_t n) { sent.push_back(n); return true; });
    EXPECT_EQ(sent, (std::vector<uint8_t>{1}));
}

TEST(MappingMachineTest, EvenEchoesAreUnconditionalInEveryState) {
    for (auto state : states) {
        auto machine = inState(state);
        for (int i = 0; i < 5; ++i) machine.tick(0, [](uint8_t) { return true; });
        for (uint8_t number : {2, 2, 4, 4}) machine.receivePing(number, 1);
        std::vector<uint8_t> sent;
        for (int i = 0; i < 4; ++i)
            machine.tick(1, [&](uint8_t n) { sent.push_back(n); return true; });
        EXPECT_EQ(sent, (std::vector<uint8_t>{2, 2, 4, 4}));
    }
}

TEST(MappingMachineTest, OddRetriesWaitTenSecondsFromSuccessfulSendAndCompleteOnce) {
    Machine machine;
    machine.requestStart();
    std::vector<uint8_t> sent;
    auto send = [&](uint8_t n) { sent.push_back(n); return true; };
    machine.tick(0, [](uint8_t) { return false; });
    EXPECT_TRUE(machine.hasPendingPing());
    machine.receivePing(1, 1);  // Unsolicited ACK before any accepted send.
    machine.tick(50, send);
    machine.tick(10049, send);
    EXPECT_EQ(sent.size(), 1u);
    machine.tick(10050, send);
    EXPECT_EQ(sent, (std::vector<uint8_t>{1, 1}));
    machine.receivePing(1, 10051);
    machine.tick(10051, [](uint8_t) { return false; });
    EXPECT_TRUE(machine.hasPendingPing());
    machine.tick(10052, send);
    machine.receivePing(1, 10053);
    machine.tick(10053, send);
    machine.tick(100000, send);
    EXPECT_EQ(sent, (std::vector<uint8_t>{1, 1, 1, 1}));
    EXPECT_EQ(machine.getState(), State::Setup);
    EXPECT_FALSE(machine.hasPendingPing());
}

TEST(MappingMachineTest, SaveIncompleteIsImmediateAndCannotOverlapOrRepeatWorkOnAck) {
    auto machine = inState(State::Mapping);
    std::vector<uint8_t> sent;
    auto send = [&](uint8_t n) { sent.push_back(n); return true; };
    for (int i = 0; i < 5; ++i) machine.tick(0, send);
    machine.requestSave();
    EXPECT_EQ(machine.getState(), State::Mapping);
    machine.tick(10, send);
    machine.receivePing(2, 11);
    machine.requestSave();
    EXPECT_EQ(machine.getState(), State::Done);
    machine.receivePing(3, 11);
    machine.tick(11, send);
    machine.tick(11, send);
    machine.tick(10010, send);
    EXPECT_EQ(std::count(sent.begin(), sent.end(), 3), 2);
    machine.tick(10011, send);
    machine.tick(10011, send);
    EXPECT_EQ(std::count(sent.begin(), sent.end(), 3), 3);
    machine.receivePing(3, 10012);
    EXPECT_EQ(machine.getState(), State::Done);
}

TEST(MappingMachineTest, OddRetryAndReadyCadenceSurviveClockRollover) {
    auto machine = inState(State::Mapping);
    const uint32_t start = std::numeric_limits<uint32_t>::max() - 500;
    for (int i = 0; i < 5; ++i) machine.tick(start, [](uint8_t) { return true; });
    machine.receivePing(1, start);
    machine.tick(start, [](uint8_t) { return true; });
    machine.requestSave();
    int sends = 0;
    auto send = [&](uint8_t n) { if (n == 3) ++sends; return true; };
    machine.tick(start, send);
    machine.tick(start + 9999, send);
    EXPECT_EQ(sends, 1);
    machine.tick(start + 10000, send);
    EXPECT_EQ(sends, 2);
    machine.receivePing(2, start);
    EXPECT_TRUE(machine.readyBeep(start + 1000));
    EXPECT_FALSE(machine.readyBeep(start + 1100));
}

TEST(MappingDriveTest, CapsMagnitudeAndRotationWithoutBoostAndStopsOnDisconnect) {
    for (float right : {-2.0f, -1.0f, -0.1f, 0.0f, 0.5f, 1.0f, 2.0f}) {
        for (float forward : {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f}) {
            const auto drive = robots::mappingDrive(right, forward, 2, true);
            EXPECT_LE(std::hypot(drive.getX(), drive.getY()), 0.400001f);
            EXPECT_FLOAT_EQ(drive.getRotation(), 0.5f);
            EXPECT_EQ(robots::mappingDrive(right, forward, 1, false), Pose2d(0, 0, 0));
        }
    }
    EXPECT_FLOAT_EQ(robots::mappingDrive(0, 0.5f, -0.5f, true).getY(), 0.2f);
    EXPECT_EQ(robots::mappingDrive(std::numeric_limits<float>::quiet_NaN(), 0, 0, true), Pose2d(0, 0, 0));
}
