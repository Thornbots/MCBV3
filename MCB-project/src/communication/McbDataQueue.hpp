// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace communication {
// Wire contract and timing derivation: ros2_dji_serial_bridge/UART_PROTOCOL.md.
// Main scheduler context only: neither enqueue nor drain is ISR/thread safe.
class McbDataQueue {
public:
    static constexpr size_t MAX_DATA = 48;
    static constexpr size_t CAPACITY = 4;
    static constexpr uint8_t TEXT_CHANNEL = 0;
    static constexpr uint8_t DROPS_CHANNEL = 255;
    struct Packet {
        std::array<uint8_t, MAX_DATA + 1> bytes{};
        uint16_t size = 1;
    };

    bool enqueue(uint8_t channel, const void* data, size_t length) {
        if (channel == DROPS_CHANNEL || length > MAX_DATA || (length && !data) || count == CAPACITY) {
            if (drops != std::numeric_limits<uint32_t>::max()) ++drops;
            return false;
        }
        auto& packet = packets[(head + count) % CAPACITY];
        packet.bytes[0] = channel;
        if (length) std::memcpy(packet.bytes.data() + 1, data, length);
        packet.size = static_cast<uint16_t>(length + 1);
        ++count;
        return true;
    }

    bool peek(Packet& packet) const {
        if (drops != reportedDrops && (!count || !lastWasReport)) {
            packet.bytes[0] = DROPS_CHANNEL;
            for (unsigned i = 0; i < 4; ++i) packet.bytes[i + 1] = drops >> (8 * i);
            packet.size = 5;
            return true;
        }
        if (!count) return false;
        packet = packets[head];
        return true;
    }

    void consume(const Packet& packet) {
        lastWasReport = packet.bytes[0] == DROPS_CHANNEL;
        if (lastWasReport) {
            reportedDrops = 0;
            for (unsigned i = 0; i < 4; ++i) reportedDrops |= static_cast<uint32_t>(packet.bytes[i + 1]) << (8 * i);
        } else if (count) {
            head = (head + 1) % CAPACITY;
            --count;
        }
    }
    uint32_t dropped() const { return drops; }
    size_t freeSlots() const { return CAPACITY - count; }
    static constexpr uint32_t wireMilliseconds(size_t frameBytes) {
        return static_cast<uint32_t>((frameBytes * 10000 + 115199) / 115200);
    }
    static bool fitsBefore(uint32_t now, uint32_t deadline, size_t frameBytes) {
        return static_cast<int32_t>(deadline - now) >= static_cast<int32_t>(wireMilliseconds(frameBytes) + 1);
    }

private:
    std::array<Packet, CAPACITY> packets{};
    size_t head = 0, count = 0;
    uint32_t drops = 0, reportedDrops = 0;
    bool lastWasReport = false;
};
}
