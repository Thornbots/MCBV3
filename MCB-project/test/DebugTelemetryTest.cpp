// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include <algorithm>
#include <sys/socket.h>
#include <unistd.h>
#include <memory>
#include <limits>
#include <vector>
#include "hosted/clock.hpp"
#include "hosted/uart.hpp"
#include "robots/RobotControl.hpp"
#include "util/CanBusHealthTelemetry.hpp"

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

#ifdef MCB_CAN_BUS_HEALTH
TEST_F(DebugTelemetryTest, CanHealthCountsVirtualFeedbackAndDropsBusyWindows) {
    auto& health = drivers->canHealth;
    health.start(0);
    modm::can::Message message(0x207, 8);
    tap::motor::DjiMotor& yaw = control->hardware.yawMotor;
    yaw.processMessage(message);
    tap::motor::DjiMotor& pitch = control->hardware.pitchMotor;
    modm::can::Message pitchMessage(0x205, 8);
    pitch.processMessage(pitchMessage);
    auto payload = health.finish(1000, {}, false);
    EXPECT_EQ(payload[45], 1); // CAN1 ID7
    EXPECT_EQ(payload[82], 1); // CAN2 ID5
    EXPECT_TRUE(yaw.isMotorOnline()); // Taproot processing still runs.
    EXPECT_EQ(payload[57], 0); // Hosted ESR unavailable.

    ASSERT_TRUE(control->jetson.sendDebug(nullptr, 0));
    hosted::timeMs = 2000;
    src::reportCanBusHealth(*drivers, control->jetson);
    EXPECT_FALSE(health.due(2000)); // Busy slot still resets the window.
    const auto next = health.finish(3000, {}, false);
    EXPECT_EQ(next[1], 2); // Lost window has its own sequence.
}
#endif

TEST_F(DebugTelemetryTest, LengthBoundsCopyAndSinglePendingSlot) {
    auto& jetson = control->jetson;
    uint8_t data[129]{};
    EXPECT_FALSE(jetson.sendDebug(data, 129));
    EXPECT_FALSE(jetson.sendDebug(nullptr, 1));
    for (size_t i = 0; i < 128; ++i) data[i] = i;
    ASSERT_TRUE(jetson.sendDebug(data, 128));
    EXPECT_FALSE(jetson.sendDebug(data, 1));
    data[0] = 99;
    EXPECT_TRUE(output().empty());  // API only copies, never writes.
    jetson.refresh();
    auto bytes = output();
    ASSERT_EQ(bytes.size(), 137U);
    EXPECT_EQ(bytes[5], 6);
    for (size_t i = 0; i < 128; ++i) EXPECT_EQ(bytes[i + 7], i);
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
        uint8_t data[128]{};
        uint32_t poses = 0, refs = 0, debug = 0, mandatory = 0;
        uint32_t delayedByDebug = 0, lastPose = 0, maxPoseGap = 0;
        uint64_t wireFreeUs = 0;
        for (uint32_t elapsed = 0; elapsed <= 6010; ++elapsed) {
            hosted::timeMs = phase + elapsed;
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
            const auto bytes = output();
            for (size_t offset = 0; offset < bytes.size();) {
                ASSERT_GE(bytes.size() - offset, 9U);
                const size_t length = bytes[offset + 1] | bytes[offset + 2] << 8;
                ASSERT_LE(offset + length + 9, bytes.size());
                const auto type = bytes[offset + 5];
                EXPECT_GE(elapsed * 1000ULL, wireFreeUs);
                wireFreeUs = elapsed * 1000ULL +
                    communication::UARTCommunication::wireTimeUs(length + 9);
                if (type == subsystems::DEBUG) {
                    ++debug;
                    EXPECT_EQ(length, 128U);
                    EXPECT_LE(delayedByDebug, 1U);
                    delayedByDebug = 0;
                    // Both frames, including two scheduler service intervals, fit before slot two.
                    EXPECT_LE(wireFreeUs +
                        communication::UARTCommunication::wireTimeUs(sizeof(subsystems::Pose) + 9) + 2000,
                        (mandatory + 2U) * 10000ULL);
                } else {
                    ++mandatory;
                    const uint32_t deadline = mandatory * 10;
                    EXPECT_GE(elapsed, deadline);
                    EXPECT_LE(elapsed - deadline, 10U);
                    if (elapsed > deadline) ++delayedByDebug;
                    EXPECT_LE(delayedByDebug, 1U);
                    EXPECT_EQ(type, mandatory % 10U == 0 ? subsystems::REF_SYS : subsystems::POSE);
                    if (type == subsystems::POSE) {
                        if (poses) maxPoseGap = std::max(maxPoseGap, elapsed - lastPose);
                        lastPose = elapsed;
                        if (mandatory <= 600) ++poses;
                    }
                    if (type == subsystems::REF_SYS && mandatory <= 600) ++refs;
                }
                offset += length + 9;
            }
        }
        EXPECT_GE(mandatory, 600U);
        EXPECT_EQ(poses, 540U);
        EXPECT_EQ(refs, 60U);
        EXPECT_LE(maxPoseGap, 30U);
        EXPECT_GT(debug, 200U); // No fixed 100 ms throttle; consume available slack.

    }
}

TEST_F(DebugTelemetryTest, DebugWaitsNearSecondDeadlineAndPingWins) {
    uint8_t data[128]{};
    hosted::timeMs = 6;
    ASSERT_TRUE(control->jetson.sendDebug(data, sizeof(data)));
    control->jetson.refresh();
    EXPECT_TRUE(output().empty()); // Insufficient budget before the second deadline.
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

TEST_F(DebugTelemetryTest, WireDrainUsesEnqueueTimeEvenWhenBufferEmptiesImmediately) {
    uint8_t bytes[128]{};
    ASSERT_TRUE(drivers->uart.sendMsg(bytes, subsystems::DEBUG, sizeof(bytes)));
    EXPECT_TRUE(drivers->uart.isFinishedWriting());
    hosted::timeMs = 3;
    EXPECT_FALSE(drivers->uart.isWireIdle());
    hosted::timeMs = 11;
    EXPECT_FALSE(drivers->uart.isWireIdle());
    hosted::timeMs = 12;
    EXPECT_TRUE(drivers->uart.isWireIdle());
    ASSERT_TRUE(drivers->uart.sendMsg(bytes, subsystems::PING, 1));
    hosted::timeMs = 14;
    EXPECT_TRUE(drivers->uart.isWireIdle()); // Late observation adds no extra frame guard.
    hosted::timeMs = 15;
    EXPECT_TRUE(drivers->uart.isWireIdle());
}

TEST_F(DebugTelemetryTest, FailedDebugIsDroppedAndStillChargedAgainstBudget) {
    uint8_t data[128]{};
    uint8_t fill[4096]{};
    while (write(sockets[0], fill, sizeof(fill)) > 0) {}
    const uint32_t failures = hosted::failedUartWrites();
    ASSERT_TRUE(control->jetson.sendDebug(data, sizeof(data)));
    control->jetson.refresh();
    EXPECT_GT(hosted::failedUartWrites(), failures);
    output();
    ASSERT_TRUE(control->jetson.sendDebug(data, sizeof(data)));
    bool debugSent = false;
    for (hosted::timeMs = 1; hosted::timeMs <= 40; ++hosted::timeMs) {
        control->jetson.refresh();
        auto bytes = output();
        if (!bytes.empty() && bytes[5] == subsystems::DEBUG) {
            EXPECT_GE(hosted::timeMs, 12U);
            debugSent = true;
        }
    }
    EXPECT_TRUE(debugSent);
}

TEST_F(DebugTelemetryTest, NoProducerSendsOnlyTelemetry) {
    for (hosted::timeMs = 0; hosted::timeMs <= 100; ++hosted::timeMs) {
        control->jetson.refresh();
        auto bytes = output();
        if (!bytes.empty()) { EXPECT_NE(bytes[5], subsystems::DEBUG); }
    }
}

TEST_F(DebugTelemetryTest, CalibrationPauseStartsFreshCycleAcrossRollover) {
    for (uint32_t phase : {0U, 0xFFFFFFD0U}) {
        reset(phase);
        for (uint32_t elapsed = 0; elapsed <= 37; ++elapsed) {
            hosted::timeMs = phase + elapsed;
            control->jetson.refresh();
            output();
        }
        const uint32_t resume = phase + 4037U;
        for (uint32_t elapsed = 0; elapsed <= 100; ++elapsed) {
            hosted::timeMs = resume + elapsed;
            control->jetson.refresh();
            const auto bytes = output();
            if (elapsed == 0 || elapsed % 10 != 0) {
                EXPECT_TRUE(bytes.empty()) << "resume elapsed=" << elapsed;
            } else {
                ASSERT_GE(bytes.size(), 9U);
                const bool referee = elapsed == 100;
                EXPECT_EQ(bytes.size(), referee ? 20U : 34U);
                EXPECT_EQ(bytes[5], referee ? subsystems::REF_SYS : subsystems::POSE);
            }
        }
    }
}

TEST_F(DebugTelemetryTest, BootCalibrationStartsWithFreshPoseDeadline) {
    hosted::timeMs = 4000;
    control->jetson.refresh();
    EXPECT_TRUE(output().empty());
    hosted::timeMs = 4009;
    control->jetson.refresh();
    EXPECT_TRUE(output().empty());
    hosted::timeMs = 4010;
    control->jetson.refresh();
    const auto bytes = output();
    ASSERT_EQ(bytes.size(), 34U);
    EXPECT_EQ(bytes[5], subsystems::POSE);
}

TEST_F(DebugTelemetryTest, OnlyGapsLongerThanOneCycleResetDeadlines) {
    for (uint32_t phase : {0U, 0xFFFFFFD0U}) {
        for (uint32_t gap : {100U, 101U}) {
            reset(phase);
            hosted::timeMs = phase + gap;
            control->jetson.refresh();
            const auto bytes = output();
            if (gap == 100U) {
                ASSERT_EQ(bytes.size(), 34U);
                EXPECT_EQ(bytes[5], subsystems::POSE);
            } else {
                EXPECT_TRUE(bytes.empty());
            }
        }
    }
}

TEST(AimWire, StructSizesAndOffsetsMatchTheContract) {
    EXPECT_EQ(sizeof(subsystems::CvTarget), 13U);
    EXPECT_EQ(offsetof(subsystems::CvTarget, flags), 12U);
    EXPECT_EQ(subsystems::CV_TARGET_FLAG_TYPE_C_BASED_PATROL, 0x02);
    EXPECT_EQ(subsystems::CV_TARGET_FLAG_TURN_TO_HIT, 0x04);
    using TS = subsystems::TargetState;
    EXPECT_EQ(sizeof(TS), 80U);
    EXPECT_EQ(offsetof(TS, center), 0U);
    EXPECT_EQ(offsetof(TS, velocity), 12U);
    EXPECT_EQ(offsetof(TS, acceleration), 24U);
    EXPECT_EQ(offsetof(TS, yaw), 36U);
    EXPECT_EQ(offsetof(TS, yaw_rate), 40U);
    EXPECT_EQ(offsetof(TS, radius), 44U);
    EXPECT_EQ(offsetof(TS, z_offset), 52U);
    EXPECT_EQ(offsetof(TS, panel), 60U);
    EXPECT_EQ(offsetof(TS, state_age_ms), 72U);
    EXPECT_EQ(offsetof(TS, detection_age_ms), 74U);
    EXPECT_EQ(offsetof(TS, track_id), 76U);
    EXPECT_EQ(offsetof(TS, confidence), 78U);
    EXPECT_EQ(offsetof(TS, flags), 79U);
    EXPECT_EQ(subsystems::TARGET_STATE, 7);
}

TEST_F(DebugTelemetryTest, TargetStateIsReceivedWithItsReceiptTime) {
    subsystems::TargetState sent{};
    sent.center[0] = 3.5f;
    sent.state_age_ms = 12;
    sent.track_id = 9;
    sent.confidence = 200;
    sent.flags = subsystems::TARGET_STATE_FLAG_VALID;
    hosted::timeMs = 1234;
    receive(subsystems::TARGET_STATE, sent);
    hosted::timeMs = 1236;
    subsystems::CvTarget wrongType{};
    EXPECT_FALSE(control->jetson.getCvTarget(&wrongType));  // not consumed by another type
    subsystems::TargetState got{};
    uint32_t receivedMs = 0;
    ASSERT_TRUE(control->jetson.getTargetState(&got, &receivedMs));
    EXPECT_FLOAT_EQ(got.center[0], 3.5f);
    EXPECT_EQ(got.state_age_ms, 12);
    EXPECT_EQ(got.track_id, 9);
    EXPECT_EQ(receivedMs, 1234U);
    EXPECT_FALSE(control->jetson.getTargetState(&got, &receivedMs));
}

TEST_F(DebugTelemetryTest, AutoAimHoldsAPlannedTargetStateAndDropsItAtTimeout) {
    subsystems::TargetState ts{};
    ts.center[0] = 3.0f; ts.center[2] = 0.3f;
    ts.yaw = 3.14159265f;
    ts.radius[0] = ts.radius[1] = 0.3f;
    ts.panel[0] = 9.0f;
    ts.detection_age_ms = 5;
    ts.track_id = 1;
    ts.confidence = 255;
    ts.flags = subsystems::TARGET_STATE_FLAG_VALID;
    auto& cmd = control->autoFire;
    cmd.initialize();
    hosted::timeMs = 1000;
    receive(subsystems::TARGET_STATE, ts);
    cmd.execute();
    EXPECT_TRUE(cmd.holdingTargetState);
    EXPECT_NEAR(cmd.deltaX, 2.7f, 0.05f);  // the facing panel, not ts.panel
    EXPECT_NEAR(cmd.deltaY, 0.0f, 0.05f);
    EXPECT_FALSE(cmd.everRequestedFire);  // measured gimbal has not acquired this panel
    // The link deadline uses receipt time, independently of carried ages.
    hosted::timeMs = 1599;
    cmd.execute();
    EXPECT_TRUE(cmd.holdingTargetState);
    hosted::timeMs = 1600;
    cmd.execute();
    EXPECT_FALSE(cmd.holdingTargetState);
}

TEST_F(DebugTelemetryTest, AutoAimClearOverridesValidAndMalformedGeometryImmediately) {
    subsystems::TargetState ts{};
    ts.center[0]=3; ts.radius[0]=ts.radius[1]=.25f;
    ts.flags=subsystems::TARGET_STATE_FLAG_VALID;
    auto& cmd=control->autoFire;
    cmd.initialize();
    receive(subsystems::TARGET_STATE,ts); cmd.execute();
    ASSERT_TRUE(cmd.holdingTargetState);
    ts.flags |= subsystems::TARGET_STATE_FLAG_CLEAR;
    ts.center[0]=std::numeric_limits<float>::quiet_NaN();
    receive(subsystems::TARGET_STATE,ts); cmd.execute();
    EXPECT_FALSE(cmd.holdingTargetState);
    cmd.end(false); cmd.initialize(); cmd.execute();
    EXPECT_FALSE(cmd.holdingTargetState);
}

TEST_F(DebugTelemetryTest, TargetMailboxSurvivesIdleTrafficAndDeliversClearBeforeReassignment) {
    subsystems::TargetState state{};
    state.flags=subsystems::TARGET_STATE_FLAG_CLEAR;
    hosted::timeMs=10;
    receive(subsystems::TARGET_STATE,state);
    state.flags=subsystems::TARGET_STATE_FLAG_VALID;
    state.track_id=3;
    hosted::timeMs=11;
    receive(subsystems::TARGET_STATE,state);
    subsystems::CvTarget idle{};
    hosted::timeMs=12;
    receive(subsystems::CV_TARGET,idle);
    subsystems::TargetState got{};
    uint32_t receipt=0;
    ASSERT_TRUE(control->jetson.getTargetState(&got,&receipt));
    EXPECT_NE(got.flags & subsystems::TARGET_STATE_FLAG_CLEAR,0);
    EXPECT_EQ(receipt,10U);
    ASSERT_TRUE(control->jetson.getTargetState(&got,&receipt));
    EXPECT_EQ(got.track_id,3);
    EXPECT_EQ(receipt,11U);
    EXPECT_FALSE(control->jetson.getTargetState(&got,&receipt));
    EXPECT_TRUE(control->jetson.getCvTarget(&idle));
}
