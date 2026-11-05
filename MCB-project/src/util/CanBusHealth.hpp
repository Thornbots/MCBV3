// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace src {
// Cooperative main-loop counters; the wire layout is owned by README's debug section.
class CanBusHealth {
public:
    using Payload = std::array<uint8_t, 103>;

    void start(uint32_t now) { startMs = now; cycles = 0; buses = {}; }
    void sample(bool ready1, bool ready2) {
        ++cycles;
        buses[0].skipped += !ready1;
        buses[1].skipped += !ready2;
    }
    void feedback(size_t bus, uint32_t identifier) {
        if (bus < buses.size() && identifier >= 0x201 && identifier <= 0x208)
            ++buses[bus].feedback[identifier - 0x201];
    }
    bool due(uint32_t now) const { return static_cast<uint32_t>(now - startMs) >= 1000U; }

    Payload finish(uint32_t now, const std::array<uint32_t, 2>& esr, bool esrValid) {
        Payload result{};
        size_t offset = 0;
        result[offset++] = 1;
        const auto put32 = [&](uint32_t value) {
            for (unsigned byte = 0; byte < 4; ++byte) result[offset++] = value >> (8 * byte);
        };
        put32(sequence++);
        put32(now - startMs);
        put32(cycles);
        for (size_t bus = 0; bus < buses.size(); ++bus) {
            put32(buses[bus].skipped);
            put32(0); // sendMessage failures are not exposed by Taproot; validity bit stays clear.
            for (auto count : buses[bus].feedback) put32(count);
            put32(esr[bus]);
            result[offset++] = esrValid ? 1 : 0;
        }
        start(now);
        return result;
    }

private:
    struct Bus {
        uint32_t skipped = 0;
        std::array<uint32_t, 8> feedback{};
    };
    std::array<Bus, 2> buses{};
    uint32_t startMs = 0;
    uint32_t cycles = 0;
    uint32_t sequence = 0;
};
} // namespace src
