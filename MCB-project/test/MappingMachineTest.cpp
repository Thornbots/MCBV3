// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include <algorithm>
#include <deque>
#include <limits>
#include <vector>
#include "util/MappingMachine.hpp"
#include "subsystems/jetson/MappingDrive.hpp"

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

// Jetson ACKs every odd frame; its sliding quiet window gates only work.
struct JetsonPeer {
    std::array<int, 2> work{};
    std::array<int64_t, 2> lastReceive{-10000, -10000};
    bool awaiting = false;
    bool started = false;
    template<typename Send>
    void receive(uint8_t number, uint32_t now, Send send) {
        if (number == 1 || number == 3) {
            send(number);
            auto i = number / 2;
            if (now - lastReceive[i] >= 5000 && (number != 1 || !started)) {
                ++work[i];
                if (number == 1) started = true;
            }
            lastReceive[i] = now;
        } else if (awaiting) {
            awaiting = false;
            send(number);
        }
    }
};
}

TEST(MappingMachineTest, AlwaysAckJetsonTerminatesWithAnySingleFrameDropped) {
    for (uint8_t number : {1, 2, 3, 4, 6}) {
        for (int drop = -1; drop < 6; ++drop) {
            SCOPED_TRACE(::testing::Message() << "number=" << +number << " drop=" << drop);
            Machine machine;
            JetsonPeer jetson;
            if (number != 1) {
                machine.requestStart();
                machine.tick(0, [](uint8_t) { return true; });
                machine.receivePing(1, 0);
                machine.tick(0, [](uint8_t) { return true; });
            }
            if (number == 3 || number == 4) {
                machine.receivePing(2, 0);
                machine.tick(0, [](uint8_t) { return true; });
            }
            const auto before = machine.getState();
            int transitions = 0;
            int frames = 0;
            struct Frame { bool toJetson; uint8_t number; };
            std::deque<Frame> wire;
            auto emit = [&](bool toJetson, uint8_t n) {
                if (frames++ != drop) wire.push_back({toJetson, n});
                return true;
            };
            if (number == 1) machine.requestStart();
            else if (number == 3) machine.requestSave();
            else { jetson.awaiting = true; emit(false, number); }
            auto previous = before;
            for (uint32_t now = 0; now < 6000 && frames <= 8; ++now) {
                if (machine.getState() != previous) {
                    ++transitions;
                    previous = machine.getState();
                }
                if (jetson.awaiting && now != 0 && now % 500 == 0) emit(false, number);
                if (!wire.empty()) {
                    const auto frame = wire.front();
                    wire.pop_front();
                    if (frame.toJetson) jetson.receive(frame.number, now,
                        [&](uint8_t n) { emit(false, n); });
                    else machine.receivePing(frame.number, now);
                }
                machine.tick(now, [&](uint8_t n) { return emit(true, n); });
            }
            EXPECT_LE(frames, 6);
            EXPECT_TRUE(wire.empty());
            EXPECT_FALSE(machine.hasPendingPing());
            EXPECT_FALSE(jetson.awaiting);
            EXPECT_EQ(transitions, 1);
            EXPECT_EQ(jetson.work[0], number == 1 ? 1 : 0);
            EXPECT_EQ(jetson.work[1], number == 3 ? 1 : 0);
        }
    }
}

TEST(MappingMachineTest, CloseDistinctSavesAreAckedButJetsonSuppressesWorkUntilQuiet) {
    auto machine = inState(State::Mapping);
    for (int i = 0; i < 5; ++i) machine.tick(0, [](uint8_t) { return true; });
    JetsonPeer jetson;
    int saves = 0;
    for (uint32_t start : {10, 600, 5602}) {
        machine.requestSave();
        for (uint32_t now = start; now < start + 3; ++now) {
            std::vector<uint8_t> sent;
            machine.tick(now, [&](uint8_t n) {
                sent.push_back(n);
                return true;
            });
            for (auto n : sent) {
                if (n != 3) continue;
                ++saves;
                jetson.receive(n, now, [&](uint8_t ack) { machine.receivePing(ack, now); });
            }
        }
        EXPECT_FALSE(machine.hasPendingPing());
        EXPECT_EQ(machine.getState(), State::Mapping);
    }
    EXPECT_EQ(saves, 6);  // Three requests and their final legs, with no retry loop.
    EXPECT_EQ(jetson.work[1], 2);
}

TEST(MappingMachineTest, EveryStateAndInputTransition) {
    for (auto state : states) {
        for (uint8_t input : {0, 1, 2, 3, 4, 5, 6, 7, 255}) {
            auto machine = inState(state);
            machine.receivePing(input, 100);
            auto expected = state;
            if (input == 2 && (state == State::Setup || state == State::Mapping)) expected = State::Ready;
            if (input == 4 && (state == State::Ready || state == State::Done)) expected = State::Mapping;
            if (input == 6 && state == State::Setup) expected = State::Done;
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
        for (uint8_t number : {2, 2, 4, 4, 6, 6}) machine.receivePing(number, 1);
        std::vector<uint8_t> sent;
        for (int i = 0; i < 6; ++i)
            machine.tick(1, [&](uint8_t n) { sent.push_back(n); return true; });
        EXPECT_EQ(sent, (std::vector<uint8_t>{2, 2, 4, 4, 6, 6}));
    }
}

TEST(MappingMachineTest, OddRetriesWaitHalfSecondFromSuccessfulSendAndCompleteOnce) {
    Machine machine;
    machine.requestStart();
    std::vector<uint8_t> sent;
    auto send = [&](uint8_t n) { sent.push_back(n); return true; };
    machine.tick(0, [](uint8_t) { return false; });
    EXPECT_TRUE(machine.hasPendingPing());
    machine.receivePing(1, 1);  // Unsolicited ACK before any accepted send.
    machine.tick(50, send);
    machine.tick(549, send);
    EXPECT_EQ(sent.size(), 1u);
    machine.tick(550, send);
    EXPECT_EQ(sent, (std::vector<uint8_t>{1, 1}));
    machine.receivePing(1, 551);
    machine.tick(551, [](uint8_t) { return false; });
    EXPECT_TRUE(machine.hasPendingPing());
    machine.tick(552, send);
    machine.receivePing(1, 553);
    machine.tick(553, send);
    machine.tick(100000, send);
    EXPECT_EQ(sent, (std::vector<uint8_t>{1, 1, 1}));
    EXPECT_EQ(machine.getState(), State::Setup);
    EXPECT_FALSE(machine.hasPendingPing());
}

TEST(MappingMachineTest, DoneToMappingCancelsEveryKindOfPendingSaveWork) {
    for (int pending = 0; pending < 4; ++pending) {
        SCOPED_TRACE(pending);
        Machine machine;
        machine.requestStart();
        machine.tick(0, [](uint8_t) { return true; });
        machine.receivePing(1, 0);
        machine.receivePing(2, 0);
        for (int i = 0; i < 3; ++i) machine.tick(0, [](uint8_t) { return true; });
        if (pending == 3) {
            machine.receivePing(4, 1);
            machine.requestSave();  // Queue incomplete save before the final save.
            machine.receivePing(2, 1);
        }
        machine.requestSave();
        ASSERT_EQ(machine.getState(), State::Done);
        if (pending == 1 || pending == 2) machine.tick(10, [](uint8_t) { return true; });
        if (pending == 2) machine.receivePing(3, 11);  // Final echo is still pending.
        machine.receivePing(4, 12);
        ASSERT_EQ(machine.getState(), State::Mapping);
        EXPECT_TRUE(machine.canDrive());
        std::vector<uint8_t> sent;
        auto send = [&](uint8_t n) { sent.push_back(n); return true; };
        machine.tick(12, [](uint8_t) { return false; });
        machine.receivePing(3, 13);  // Late save ACK must not resurrect the transaction.
        for (uint32_t now = 13; now < 2000; ++now) machine.tick(now, send);
        EXPECT_EQ(std::count(sent.begin(), sent.end(), 3), 0);
        EXPECT_GE(std::count(sent.begin(), sent.end(), 4), 1);
        EXPECT_FALSE(machine.hasPendingPing());
        machine.requestSave();
        machine.tick(2000, send);
        EXPECT_EQ(sent.back(), 3);  // A new operator save still works.
    }
}

TEST(MappingMachineTest, FailInSetupCancelsUnsentOrAwaitingStartAndStopsRetries) {
    for (bool sentStart : {false, true}) {
        Machine machine;
        machine.requestStart();
        if (sentStart) machine.tick(0, [](uint8_t) { return true; });
        machine.receivePing(6, 1);
        EXPECT_EQ(machine.getState(), State::Done);
        EXPECT_FALSE(machine.canDrive());
        EXPECT_FALSE(machine.readyBeep(1));
        machine.tick(1, [](uint8_t) { return false; });
        EXPECT_TRUE(machine.hasPendingPing());
        machine.receivePing(1, 2);  // Late start ACK is no longer awaited.
        machine.receivePing(6, 2);
        std::vector<uint8_t> sent;
        for (uint32_t now = 2; now < 2000; ++now)
            machine.tick(now, [&](uint8_t n) { sent.push_back(n); return true; });
        EXPECT_EQ(sent, (std::vector<uint8_t>{6, 6}));
        EXPECT_FALSE(machine.hasPendingPing());
    }
}

TEST(MappingMachineTest, FailEchoesWithoutStateChangeOutsideSetup) {
    for (auto state : {State::Main, State::Ready, State::Mapping, State::Done}) {
        auto machine = inState(state);
        for (int i = 0; i < 5; ++i) machine.tick(0, [](uint8_t) { return true; });
        std::vector<uint8_t> sent;
        for (int i = 0; i < 2; ++i) {
            machine.receivePing(6, 1);
            EXPECT_EQ(machine.getState(), state);
            machine.tick(1, [&](uint8_t n) { sent.push_back(n); return true; });
        }
        EXPECT_EQ(sent, (std::vector<uint8_t>{6, 6}));
    }
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
    machine.tick(510, send);
    EXPECT_EQ(std::count(sent.begin(), sent.end(), 3), 2);
    machine.tick(511, send);
    machine.tick(511, send);
    EXPECT_EQ(std::count(sent.begin(), sent.end(), 3), 3);
    machine.receivePing(3, 512);
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
    machine.tick(start + 499, send);
    EXPECT_EQ(sends, 1);
    machine.tick(start + 500, send);
    EXPECT_EQ(sends, 2);
    machine.receivePing(2, start);
    EXPECT_TRUE(machine.readyBeep(start + 1000));
    EXPECT_FALSE(machine.readyBeep(start + 1100));
}

TEST(MappingDriveTest, CapsMagnitudeAndRotationWithoutBoostAndStopsOnDisconnect) {
    for (float right : {-2.0f, -1.0f, -0.1f, 0.0f, 0.5f, 1.0f, 2.0f}) {
        for (float forward : {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f}) {
            const auto drive = commands::mappingDrive(right, forward, 2, true);
            EXPECT_LE(std::hypot(drive.getX(), drive.getY()), 0.400001f);
            EXPECT_FLOAT_EQ(drive.getRotation(), 0.5f);
            EXPECT_EQ(commands::mappingDrive(right, forward, 1, false), Pose2d(0, 0, 0));
        }
    }
    EXPECT_FLOAT_EQ(commands::mappingDrive(0, 0.5f, -0.5f, true).getY(), 0.2f);
    EXPECT_EQ(commands::mappingDrive(std::numeric_limits<float>::quiet_NaN(), 0, 0, true), Pose2d(0, 0, 0));
}
