// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include <sys/socket.h>
#include <unistd.h>
#include <memory>
#include <vector>
#include "hosted/clock.hpp"
#include "hosted/uart.hpp"
#include "robots/RobotControl.hpp"

namespace {
class DebugTelemetryTest : public ::testing::Test {
protected:
    std::unique_ptr<src::Drivers> drivers;
    std::unique_ptr<robots::SentryControl> control;
    int sockets[2] = {-1, -1};
    void SetUp() override { reset(0); }
    void TearDown() override {
        control.reset();
        drivers.reset();
        close(sockets[0]);
        close(sockets[1]);
        hosted::setJetsonFd(-1);
    }
    void reset(uint32_t phase) {
        control.reset();
        drivers.reset();
        if (sockets[0] >= 0) { close(sockets[0]); close(sockets[1]); }
        hosted::timeMs = phase;
        hosted::sensors = {};
        ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0, sockets), 0);
        hosted::setJetsonFd(sockets[0]);
        drivers = std::make_unique<src::Drivers>();
        control = std::make_unique<robots::SentryControl>(drivers.get());
        control->gimbal.refresh();
    }
    std::vector<uint8_t> output() {
        std::vector<uint8_t> bytes;
        uint8_t buffer[512];
        ssize_t n;
        while ((n = read(sockets[1], buffer, sizeof(buffer))) > 0)
            bytes.insert(bytes.end(), buffer, buffer + n);
        return bytes;
    }
    template<class T> void receive(uint16_t type, const T& value) {
        communication::UARTCommunication::outgoingDataFrame frame(
            sizeof(value), type, reinterpret_cast<const uint8_t*>(&value));
        ASSERT_EQ(write(sockets[1], &frame, sizeof(value) + 9),
                  static_cast<ssize_t>(sizeof(value) + 9));
        // Actual parser runs in the 5 us I/O polling loop on board and hosted main.
        for (int i = 0; i < 100; ++i) drivers->uart.updateSerial();
    }
};
}

TEST_F(DebugTelemetryTest, LengthBoundsCopyAndSinglePendingSlot) {
    auto& jetson = control->jetson;
    uint8_t data[33]{};
    EXPECT_FALSE(jetson.sendDebug(data, 33));
    EXPECT_FALSE(jetson.sendDebug(nullptr, 1));
    for (size_t i = 0; i < 32; ++i) data[i] = i;
    ASSERT_TRUE(jetson.sendDebug(data, 32));
    EXPECT_FALSE(jetson.sendDebug(data, 1));
    data[0] = 99;
    EXPECT_TRUE(output().empty());  // API only copies, never writes.
    jetson.refresh();
    auto bytes = output();
    ASSERT_EQ(bytes.size(), 41U);
    EXPECT_EQ(bytes[5], 6);
    for (size_t i = 0; i < 32; ++i) EXPECT_EQ(bytes[i + 7], i);
    EXPECT_TRUE(jetson.sendDebug(nullptr, 0));
    for (hosted::timeMs = 1; hosted::timeMs <= 110; ++hosted::timeMs) {
        jetson.refresh();
        bytes = output();
        if (!bytes.empty() && bytes[5] == 6) {
            ASSERT_EQ(bytes.size(), 9U);
            EXPECT_EQ(bytes[1], 0);
            return;
        }
    }
    FAIL() << "empty DEBUG did not drain";
}

TEST_F(DebugTelemetryTest, FloodPreservesMandatorySlotsAndReceiveAcrossPhasesAndRollover) {
    for (uint32_t phase : {0U, 1U, 7U, 997U, 0xFFFFFFD0U}) {
        reset(phase);
        uint8_t data[32]{};
        uint32_t poses = 0, refs = 0, debug = 0, lastDebug = 0;
        bool debugBefore = false;
        uint64_t wireFreeUs = 0;
        for (uint32_t elapsed = 0; elapsed <= 2000; ++elapsed) {
            hosted::timeMs = phase + elapsed;
            // A producer flood cannot overwrite or grow the pending slot.
            for (int i = 0; i < 100; ++i) control->jetson.sendDebug(data, sizeof(data));
            if (elapsed % 13 == 0) {
                subsystems::CvTarget target{};
                target.x = elapsed;
                receive(subsystems::CV_TARGET, target);
                subsystems::CvTarget got{};
                ASSERT_TRUE(control->jetson.getCvTarget(&got));
                EXPECT_FLOAT_EQ(got.x, elapsed);
            }
            if (elapsed % 17 == 0) {
                subsystems::Relocalize fix{static_cast<float>(elapsed), -2};
                receive(subsystems::RELOCALIZE, fix);
            }
            control->jetson.refresh();
            if (elapsed % 17 == 0) { EXPECT_FLOAT_EQ(control->odo.getX(), elapsed); }
            auto bytes = output();
            bool mandatory = false;
            for (size_t offset = 0; offset < bytes.size();) {
                ASSERT_GE(bytes.size() - offset, 9U);
                const size_t length = bytes[offset + 1] | bytes[offset + 2] << 8;
                ASSERT_LE(offset + length + 9, bytes.size());
                const auto type = bytes[offset + 5];
                // Model physical 115200 8N1 drain despite hosted's always-true buffer flag.
                EXPECT_GE(elapsed * 1000ULL, wireFreeUs);
                wireFreeUs = elapsed * 1000ULL +
                    communication::UARTCommunication::wireTimeUs(length + 9);
                if (type == subsystems::DEBUG) {
                    ++debug;
                    EXPECT_EQ(length, 32U);
                    if (debugBefore) { EXPECT_GE(elapsed - lastDebug, 100U); }
                    debugBefore = true;
                    lastDebug = elapsed;
                    EXPECT_LT(wireFreeUs, (elapsed / 10U + 1U) * 10000ULL);
                } else {
                    mandatory = true;
                    EXPECT_EQ(elapsed % 10U, 0U);
                    EXPECT_EQ(type, elapsed % 100U == 0 ? subsystems::REF_SYS : subsystems::POSE);
                    if (type == subsystems::POSE) ++poses;
                    if (type == subsystems::REF_SYS) ++refs;
                }
                offset += length + 9;
            }
            EXPECT_EQ(mandatory, elapsed > 0 && elapsed % 10U == 0);
        }
        EXPECT_EQ(poses, 180U);
        EXPECT_EQ(refs, 20U);
        EXPECT_GE(debug, 19U);
        EXPECT_LE(debug, 21U);
    }
}

TEST_F(DebugTelemetryTest, LatePhaseCannotAdmitFrameBeforeDeadlineAndPingWins) {
    uint8_t data[32]{};
    hosted::timeMs = 6;
    ASSERT_TRUE(control->jetson.sendDebug(data, sizeof(data)));
    control->jetson.refresh();
    EXPECT_TRUE(output().empty()); // 4 ms remaining cannot fit the guarded frame.
    drivers->mapping.receivePing(0, 6);
    hosted::timeMs = 7;
    control->jetson.refresh();
    auto bytes = output();
    ASSERT_EQ(bytes.size(), 10U);
    EXPECT_EQ(bytes[5], subsystems::PING);
    for (hosted::timeMs = 8; hosted::timeMs <= 10; ++hosted::timeMs) {
        control->jetson.refresh();
        bytes = output();
        if (hosted::timeMs < 10) { EXPECT_TRUE(bytes.empty()); }
        else { ASSERT_EQ(bytes.size(), 34U); EXPECT_EQ(bytes[5], subsystems::POSE); }
    }
}

TEST_F(DebugTelemetryTest, BufferEmptyStillRequiresAFullDrainAfterObservation) {
    uint8_t bytes[32]{};
    ASSERT_TRUE(drivers->uart.sendMsg(bytes, subsystems::DEBUG, sizeof(bytes)));
    // Hosted reports buffer-empty immediately, even though 41 bytes need 3560 us.
    EXPECT_TRUE(drivers->uart.isFinishedWriting());
    hosted::timeMs = 3;
    EXPECT_FALSE(drivers->uart.isWireIdle()); // First observation starts the guard.
    hosted::timeMs = 6;
    EXPECT_FALSE(drivers->uart.isWireIdle());
    hosted::timeMs = 7;
    EXPECT_TRUE(drivers->uart.isWireIdle());
    ASSERT_TRUE(drivers->uart.sendMsg(bytes, subsystems::PING, 1));
    EXPECT_FALSE(drivers->uart.isWireIdle());
    hosted::timeMs = 8;
    EXPECT_TRUE(drivers->uart.isWireIdle());
}

TEST_F(DebugTelemetryTest, FailedDebugIsDroppedAndStillChargedAgainstBudget) {
    uint8_t data[32]{};
    uint8_t fill[4096]{};
    while (write(sockets[0], fill, sizeof(fill)) > 0) {}
    const uint32_t failures = hosted::failedUartWrites();
    ASSERT_TRUE(control->jetson.sendDebug(data, sizeof(data)));
    control->jetson.refresh();
    EXPECT_GT(hosted::failedUartWrites(), failures);
    output(); // Free transport; next value can be accepted, but no immediate retry.
    ASSERT_TRUE(control->jetson.sendDebug(data, sizeof(data)));
    for (hosted::timeMs = 1; hosted::timeMs < 100; ++hosted::timeMs) {
        control->jetson.refresh();
        auto bytes = output();
        if (!bytes.empty()) { EXPECT_NE(bytes[5], subsystems::DEBUG); }
    }
    for (; hosted::timeMs <= 110; ++hosted::timeMs) {
        control->jetson.refresh();
        auto bytes = output();
        if (!bytes.empty() && bytes[5] == subsystems::DEBUG) return;
    }
    FAIL() << "next value did not drain after the charged interval";
}
