// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <fcntl.h>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <sys/socket.h>
#include <type_traits>
#include <unistd.h>
#include <vector>
#include "communication/OdomI2cLog.hpp"
#include "communication/UARTCommunication.hpp"
#include "hosted/clock.hpp"
#include "hosted/uart.hpp"
#include "tap/algorithms/crc.hpp"
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

class UartPipe {
public:
    UartPipe() {
        int fds[2];
        if (::socketpair(AF_UNIX, SOCK_STREAM, 0, fds) != 0) return;
        firmwareFd = fds[0];
        peerFd = fds[1];
        const int flags = ::fcntl(peerFd, F_GETFL, 0);
        ::fcntl(peerFd, F_SETFL, flags | O_NONBLOCK);
        hosted::setJetsonFd(firmwareFd);
    }
    ~UartPipe() {
        hosted::setJetsonFd(-1);
        if (firmwareFd >= 0) ::close(firmwareFd);
        if (peerFd >= 0) ::close(peerFd);
    }
    bool valid() const { return firmwareFd >= 0 && peerFd >= 0; }
    std::vector<uint8_t> readAvailable() const {
        std::vector<uint8_t> bytes;
        std::array<uint8_t, 512> buffer{};
        ssize_t count;
        while ((count = ::read(peerFd, buffer.data(), buffer.size())) > 0)
            bytes.insert(bytes.end(), buffer.begin(), buffer.begin() + count);
        return bytes;
    }
private:
    int firmwareFd = -1;
    int peerFd = -1;
};

struct DecodedFrame {
    uint16_t type;
    std::vector<uint8_t> payload;
};

struct EmittedFrame {
    uint32_t time;
    DecodedFrame frame;
};

std::vector<DecodedFrame> decodeFrames(const std::vector<uint8_t>& bytes) {
    std::vector<DecodedFrame> frames;
    size_t offset = 0;
    while (offset < bytes.size()) {
        EXPECT_GE(bytes.size() - offset, 9u);
        if (bytes.size() - offset < 9) break;
        EXPECT_EQ(bytes[offset], 0xa5);
        const size_t length = bytes[offset + 1] |
                              (static_cast<size_t>(bytes[offset + 2]) << 8);
        const size_t frameSize = 9 + length;
        EXPECT_LE(offset + frameSize, bytes.size());
        if (offset + frameSize > bytes.size()) break;
        EXPECT_EQ(bytes[offset + 4], tap::algorithms::calculateCRC8(bytes.data() + offset, 4));
        const uint16_t receivedCrc = bytes[offset + 7 + length] |
                                     (static_cast<uint16_t>(bytes[offset + 8 + length]) << 8);
        EXPECT_EQ(receivedCrc,
                  tap::algorithms::calculateCRC16(bytes.data() + offset, 7 + length));
        const uint16_t type = bytes[offset + 5] |
                              (static_cast<uint16_t>(bytes[offset + 6]) << 8);
        frames.push_back({type, {bytes.begin() + offset + 7,
                                 bytes.begin() + offset + 7 + length}});
        offset += frameSize;
    }
    return frames;
}
}

TEST(OdometryFramesTest, RelocalizationChangesNavigationButPreservesRawAim) {
    static src::Drivers drivers;
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
    EXPECT_NEAR(odometry.getAimX(), rawX + 0.5f, 1e-6f);
    EXPECT_NEAR(odometry.getAimY(), rawY - 0.3f, 1e-6f);
    EXPECT_NEAR(odometry.getX(), 1.75f, 1e-6f);
    EXPECT_NEAR(odometry.getY(), -1.05f, 1e-6f);
    odometry.relocalizeTo(-2.0f, 3.0f);
    EXPECT_FLOAT_EQ(odometry.getX(), -2.0f);
    EXPECT_FLOAT_EQ(odometry.getY(), 3.0f);
    EXPECT_NEAR(odometry.getAimX(), rawX + 0.5f, 1e-6f);
    EXPECT_NEAR(odometry.getAimY(), rawY - 0.3f, 1e-6f);
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

TEST(McbDataQueueTest, CopiesPayloadsAndPreservesFifoAcrossRingWrap) {
    communication::McbDataQueue queue;
    std::array<uint8_t, 2> bytes{0x12, 0xa5};
    ASSERT_TRUE(queue.enqueue(7, bytes.data(), bytes.size()));
    bytes = {0xff, 0xff};
    ASSERT_TRUE(queue.enqueue(8, nullptr, 0));
    communication::McbDataQueue::Packet packet;
    ASSERT_TRUE(queue.peek(packet));
    EXPECT_EQ(packet.size, 3);
    EXPECT_EQ(packet.bytes[0], 7);
    EXPECT_EQ(packet.bytes[1], 0x12);
    EXPECT_EQ(packet.bytes[2], 0xa5);
    queue.consume(packet);
    ASSERT_TRUE(queue.peek(packet));
    EXPECT_EQ(packet.size, 1);
    EXPECT_EQ(packet.bytes[0], 8);
    queue.consume(packet);

    for (uint8_t channel = 9; channel < 14; ++channel) {
        ASSERT_TRUE(queue.enqueue(channel, nullptr, 0));
        ASSERT_TRUE(queue.peek(packet));
        EXPECT_EQ(packet.bytes[0], channel);
        queue.consume(packet);
    }
    EXPECT_EQ(queue.dropped(), 0u);
}

TEST(McbDataQueueTest, RejectsInvalidInputsAndCountsEveryDrop) {
    communication::McbDataQueue queue;
    std::array<uint8_t, communication::McbDataQueue::MAX_DATA + 1> tooLarge{};
    EXPECT_FALSE(queue.enqueue(1, nullptr, 1));
    EXPECT_FALSE(queue.enqueue(communication::McbDataQueue::DROPS_CHANNEL, nullptr, 0));
    EXPECT_FALSE(queue.enqueue(2, tooLarge.data(), tooLarge.size()));
    for (uint8_t channel = 10; channel < 14; ++channel)
        ASSERT_TRUE(queue.enqueue(channel, nullptr, 0));
    EXPECT_FALSE(queue.enqueue(14, nullptr, 0));
    EXPECT_EQ(queue.dropped(), 4u);
}

TEST(McbDataQueueTest, DropReportsDoNotStarveQueuedDataDuringOverflow) {
    communication::McbDataQueue queue;
    for (size_t index = 0; index < communication::McbDataQueue::CAPACITY; ++index)
        ASSERT_TRUE(queue.enqueue(static_cast<uint8_t>(index + 1), nullptr, 0));
    EXPECT_FALSE(queue.enqueue(20, nullptr, 0));
    communication::McbDataQueue::Packet packet;
    std::vector<uint8_t> delivered;
    bool sawCurrentReport = false;
    for (int iteration = 0; iteration < 32 && delivered.size() < 4; ++iteration) {
        ASSERT_TRUE(queue.peek(packet));
        if (packet.bytes[0] == communication::McbDataQueue::DROPS_CHANNEL) {
            const uint32_t reported = packet.bytes[1] |
                                      (static_cast<uint32_t>(packet.bytes[2]) << 8) |
                                      (static_cast<uint32_t>(packet.bytes[3]) << 16) |
                                      (static_cast<uint32_t>(packet.bytes[4]) << 24);
            EXPECT_GE(reported, 1u);
            sawCurrentReport = true;
        } else {
            delivered.push_back(packet.bytes[0]);
        }
        const bool wasReport = packet.bytes[0] == communication::McbDataQueue::DROPS_CHANNEL;
        queue.consume(packet);
        if (!wasReport) { ASSERT_TRUE(queue.enqueue(21, nullptr, 0)); }
        EXPECT_FALSE(queue.enqueue(22, nullptr, 0));
    }
    EXPECT_TRUE(sawCurrentReport);
    EXPECT_EQ(delivered, (std::vector<uint8_t>{1, 2, 3, 4}));
    ASSERT_TRUE(queue.peek(packet));
    ASSERT_EQ(packet.bytes[0], communication::McbDataQueue::DROPS_CHANNEL);
    const uint32_t reported = packet.bytes[1] |
                              (static_cast<uint32_t>(packet.bytes[2]) << 8) |
                              (static_cast<uint32_t>(packet.bytes[3]) << 16) |
                              (static_cast<uint32_t>(packet.bytes[4]) << 24);
    EXPECT_EQ(reported, queue.dropped());
}

TEST(UARTCommunicationTest, SendsBoundedOpaqueAndTextFramesWithValidChecksums) {
    UartPipe pipe;
    ASSERT_TRUE(pipe.valid());
    hosted::timeMs = 0;
    Drivers drivers;
    communication::UARTCommunication uart(
        &drivers, tap::communication::serial::Uart::Uart1, true);
    std::array<uint8_t, communication::McbDataQueue::MAX_DATA> opaque{};
    const std::array<uint8_t, 3> typed{0x34, 0x12, 0xa5};
    for (size_t i = 0; i < opaque.size(); ++i) opaque[i] = static_cast<uint8_t>(i * 17);
    ASSERT_TRUE(uart.mcbLog(6, opaque.data(), opaque.size()));
    ASSERT_TRUE(uart.mcbText("%060d", 7));
    ASSERT_TRUE(uart.mcbLog(43, typed));
    uart.sendIdle(100);
    hosted::timeMs += 6;
    uart.sendIdle(100);
    EXPECT_FALSE(uart.mcbLog(communication::McbDataQueue::DROPS_CHANNEL, nullptr, 0));
    hosted::timeMs += 6;
    uart.sendIdle(100);
    hosted::timeMs += 6;
    uart.sendIdle(100);

    const auto frames = decodeFrames(pipe.readAvailable());
    ASSERT_EQ(frames.size(), 4u);
    EXPECT_EQ(frames[0].type, 6);
    ASSERT_EQ(frames[0].payload.size(), opaque.size() + 1);
    EXPECT_EQ(frames[0].payload[0], 6);
    EXPECT_TRUE(std::equal(opaque.begin(), opaque.end(), frames[0].payload.begin() + 1));
    EXPECT_EQ(frames[1].type, 6);
    ASSERT_EQ(frames[1].payload.size(), communication::McbDataQueue::MAX_DATA + 1);
    EXPECT_EQ(frames[1].payload[0], communication::McbDataQueue::TEXT_CHANNEL);
    EXPECT_EQ(std::string(frames[1].payload.begin() + 1, frames[1].payload.end()),
              std::string(48, '0'));
    EXPECT_EQ(frames[2].type, 6);
    ASSERT_EQ(frames[2].payload.size(), 5u);
    EXPECT_EQ(frames[2].payload[0], communication::McbDataQueue::DROPS_CHANNEL);
    EXPECT_EQ(frames[2].payload[1], 1);
    EXPECT_EQ(frames[3].type, 6);
    ASSERT_EQ(frames[3].payload.size(), typed.size() + 1);
    EXPECT_EQ(frames[3].payload[0], 43);
    EXPECT_TRUE(std::equal(typed.begin(), typed.end(), frames[3].payload.begin() + 1));
}

TEST(UARTCommunicationTest, IdleGapBoundariesAndClockRolloverAreInclusive) {
    UartPipe pipe;
    ASSERT_TRUE(pipe.valid());
    hosted::timeMs = 0;
    Drivers drivers;
    communication::UARTCommunication uart(
        &drivers, tap::communication::serial::Uart::Uart1, true);
    ASSERT_TRUE(uart.mcbLog(9, nullptr, 0));
    uart.sendIdle(1);
    EXPECT_TRUE(pipe.readAvailable().empty());
    uart.sendIdle(2);
    auto first = decodeFrames(pipe.readAvailable());
    ASSERT_EQ(first.size(), 1u);
    EXPECT_EQ(first[0].payload[0], 9);

    hosted::timeMs = std::numeric_limits<uint32_t>::max() - 1;
    communication::UARTCommunication wrappingUart(
        &drivers, tap::communication::serial::Uart::Uart1, true);
    ASSERT_TRUE(wrappingUart.mcbLog(10, nullptr, 0));
    wrappingUart.sendIdle(0);
    auto beforeWrap = decodeFrames(pipe.readAvailable());
    ASSERT_EQ(beforeWrap.size(), 1u);
    EXPECT_EQ(beforeWrap[0].payload[0], 10);

    hosted::timeMs = 0;
    ASSERT_TRUE(wrappingUart.mcbLog(11, nullptr, 0));
    wrappingUart.sendIdle(1);
    EXPECT_TRUE(pipe.readAvailable().empty());
    wrappingUart.sendIdle(2);
    auto afterWrap = decodeFrames(pipe.readAvailable());
    ASSERT_EQ(afterWrap.size(), 1u);
    EXPECT_EQ(afterWrap[0].payload[0], 11);
}

TEST(UARTCommunicationTest, TelemetryAndPingKeepTheirSlotsUnderContinuousDataLoad) {
    UartPipe pipe;
    ASSERT_TRUE(pipe.valid());
    hosted::timeMs = 0;
    Drivers drivers;
    communication::UARTCommunication uart(
        &drivers, tap::communication::serial::Uart::Uart1, true);
    std::vector<EmittedFrame> frames;
    std::array<uint8_t, communication::McbDataQueue::MAX_DATA> payload{};
    uint32_t rejected = 0;
    uint32_t nextTelemetry = 9;
    uint32_t busyUntil = 0;
    bool havePreviousFrame = false;
    for (uint32_t tick = 0; tick < 100; ++tick) {
        hosted::timeMs = tick;
        if (!uart.mcbLog(42, payload.data(), payload.size())) ++rejected;
        if (tick == 8 || tick == 9 || tick == 18 || tick == 19) uart.queuePing(tick);
        if (tick == nextTelemetry) {
            uint8_t telemetry[25]{};
            const uint16_t type = (tick == 99) ? 3 : 2;
            ASSERT_TRUE(uart.sendMsg(telemetry, type, type == 2 ? 25 : 11));
            nextTelemetry += 10;
        }
        uart.sendIdle(nextTelemetry);
        auto written = decodeFrames(pipe.readAvailable());
        ASSERT_LE(written.size(), 1u);
        if (!written.empty()) {
            if (havePreviousFrame) {
                EXPECT_GE(static_cast<int32_t>(tick - busyUntil), 0);
            }
            busyUntil = tick + communication::McbDataQueue::wireMilliseconds(
                                   written[0].payload.size() + 9);
            havePreviousFrame = true;
            frames.push_back({tick, written[0]});
        }
    }

    EXPECT_GT(rejected, 0u);
    EXPECT_EQ(uart.droppedData(), rejected);
    const auto poseCount = std::count_if(frames.begin(), frames.end(), [](const auto& event) {
        return event.frame.type == 2;
    });
    const auto refCount = std::count_if(frames.begin(), frames.end(), [](const auto& event) {
        return event.frame.type == 3;
    });
    const auto pingCount = std::count_if(frames.begin(), frames.end(), [](const auto& event) {
        return event.frame.type == 5;
    });
    EXPECT_EQ(poseCount, 9);
    EXPECT_EQ(refCount, 1);
    EXPECT_GE(pingCount, 1);
    for (const auto& event : frames) {
        if (event.frame.type == 2) { EXPECT_EQ(event.frame.payload.size(), 25u); }
        if (event.frame.type == 3) { EXPECT_EQ(event.frame.payload.size(), 11u); }
    }
}

namespace {
struct OdomRun {
    std::vector<EmittedFrame> telemetry;
    std::vector<communication::OdomI2cLog::Record> records;
    std::vector<communication::OdomI2cLog::Summary> summaries;
    std::set<uint32_t> failedSeqs;
    uint32_t attempts = 0;
};

// Two Pico reads per 1 ms tick with JetsonSubsystem's POSE/REF_SYS schedule. Failures:
// a short AddressNack burst, one DataNack and a 100 ms BusBusy outage.
OdomRun runOdomSchedule(bool logEnabled) {
    UartPipe pipe;
    EXPECT_TRUE(pipe.valid());
    hosted::timeMs = 0;
    Drivers drivers;
    communication::UARTCommunication uart(&drivers, tap::communication::serial::Uart::Uart1, true);
    communication::OdomI2cLog log;
    OdomRun run;
    uint32_t nextTelemetry = 10, messageCount = 0;
    for (uint32_t tick = 0; tick < 3100; ++tick) {
        hosted::timeMs = tick;
        for (int i = 0; i < 2; ++i) {
            uint8_t result = 0;
            if (tick >= 500 && tick < 503) result = 2;
            if (tick == 1500 && i == 1) result = 3;
            if (tick >= 2000 && tick < 2100) result = 6;
            const float payload[4] = {0.001f * tick, result ? 0.0f : 0.002f * tick, 0, 0};
            log.attempt(tick * 1000 + i * 500, tick, result, payload, payload[0], payload[1]);
            ++run.attempts;
            if (result) run.failedSeqs.insert(run.attempts);
        }
        if (static_cast<int32_t>(tick - nextTelemetry) >= 0) {
            nextTelemetry += 10;
            uint8_t telemetry[25]{};
            const uint16_t type = ++messageCount < 10 ? 2 : 3;
            if (type == 3) messageCount = 0;
            EXPECT_TRUE(uart.sendMsg(telemetry, type, type == 2 ? 25 : 11));
        }
        if (logEnabled) log.drain(uart, tick * 1000);
        uart.sendIdle(nextTelemetry);
        for (auto& frame : decodeFrames(pipe.readAvailable())) {
            if (frame.type == 2 || frame.type == 3) run.telemetry.push_back({tick, frame});
            if (frame.type != 6) continue;
            if (frame.payload[0] == communication::OdomI2cLog::RECORD_CHANNEL) {
                EXPECT_EQ(frame.payload.size(), 1 + sizeof(communication::OdomI2cLog::Record));
                communication::OdomI2cLog::Record r;
                std::memcpy(&r, frame.payload.data() + 1, sizeof(r));
                run.records.push_back(r);
            } else if (frame.payload[0] == communication::OdomI2cLog::SUMMARY_CHANNEL) {
                EXPECT_EQ(frame.payload.size(), 1 + sizeof(communication::OdomI2cLog::Summary));
                communication::OdomI2cLog::Summary summary;
                std::memcpy(&summary, frame.payload.data() + 1, sizeof(summary));
                run.summaries.push_back(summary);
            }
        }
    }
    EXPECT_EQ(uart.droppedData(), 0u);
    return run;
}
}  // namespace

TEST(OdomI2cLogTest, LogsEveryFailureItCanAndKeepsTelemetrySlots) {
    const OdomRun baseline = runOdomSchedule(false);
    const OdomRun logged = runOdomSchedule(true);
    ASSERT_EQ(baseline.telemetry.size(), 309u);
    ASSERT_EQ(logged.telemetry.size(), baseline.telemetry.size());
    for (size_t i = 0; i < baseline.telemetry.size(); ++i) {
        EXPECT_EQ(logged.telemetry[i].time, baseline.telemetry[i].time);
        EXPECT_EQ(logged.telemetry[i].frame.type, baseline.telemetry[i].frame.type);
    }
    EXPECT_TRUE(baseline.records.empty());

    std::set<uint32_t> loggedFailures;
    uint32_t successes = 0, lastSuccessTime = 0;
    for (const auto& r : logged.records) {
        if (r.flags & communication::OdomI2cLog::FLAG_SUCCESS) {
            EXPECT_EQ(r.result, 0);
            if (successes++) { EXPECT_GE(r.timeUs - lastSuccessTime, 15000u); }
            lastSuccessTime = r.timeUs;
        } else {
            EXPECT_NE(r.result, 0);
            EXPECT_TRUE(logged.failedSeqs.count(r.seq));
            loggedFailures.insert(r.seq);
        }
    }
    EXPECT_GT(successes, 50u);
    // Short bursts are logged in full; the outage overflows and is counted.
    for (uint32_t seq : logged.failedSeqs)
        if (seq < 4000) { EXPECT_TRUE(loggedFailures.count(seq)) << seq; }
    ASSERT_GE(logged.summaries.size(), 2u);
    std::map<uint8_t, uint32_t> counts;
    for (const auto& s : logged.summaries)
        for (uint8_t i = 0; i < communication::OdomI2cLog::RESULT_COUNT; ++i) counts[i] += s.results[i];
    EXPECT_EQ(counts[2], 6u);
    EXPECT_EQ(counts[3], 1u);
    const auto& last = logged.summaries.back();
    EXPECT_EQ(last.fifoDrops, 0u);
    EXPECT_GT(last.lostFailures, 0u);
    EXPECT_GT(last.decimatedSuccesses, 0u);
}

TEST(OdomI2cLogTest, FreshnessFlagsLatchesWithinThresholdAndClearsOnSuccess) {
    constexpr uint32_t staleMs = 50;
    communication::OdomI2cLog log;
    const float payload[4]{};
    EXPECT_FALSE(log.succeededWithin(0, staleMs));  // no read yet
    for (uint32_t ms = 0; ms < 1000; ++ms) {
        log.attempt(ms * 1000, ms, 0, payload, 0, 0);
        EXPECT_TRUE(log.succeededWithin(ms, staleMs)) << ms;
    }
    // NACK latch: one failed read at 1000 ms, then no attempts; stuck read: no attempts.
    log.attempt(1000000, 1000, 2, payload, 0, 0);
    EXPECT_TRUE(log.succeededWithin(999 + staleMs, staleMs));
    EXPECT_FALSE(log.succeededWithin(1000 + staleMs, staleMs));
    log.attempt(2000000, 2000, 0, payload, 0, 0);
    EXPECT_TRUE(log.succeededWithin(2000, staleMs));
    // Wrap-safe across the 2^32 ms clock rollover.
    log.attempt(0, UINT32_MAX - 10, 0, payload, 0, 0);
    EXPECT_TRUE(log.succeededWithin(20, staleMs));
    EXPECT_FALSE(log.succeededWithin(40, staleMs));
}
