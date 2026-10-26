// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace communication {
// Pico odometry I2C diagnostics on MCB_DATA channels 16/17; layout and policy:
// README.md#odometry-i2c-diagnostics. Scheduler context only, not ISR safe.
class OdomI2cLog {
public:
    static constexpr uint8_t RECORD_CHANNEL = 16;
    static constexpr uint8_t SUMMARY_CHANNEL = 17;
    static constexpr uint32_t SUCCESS_PERIOD_MS = 20;  // 50 Hz fits the one data frame per POSE cycle
    static constexpr uint32_t SUMMARY_PERIOD_MS = 1000;
    static constexpr size_t FAILURE_CAPACITY = 16;
    static constexpr uint8_t RESULT_COUNT = 8;  // modm::I2cMaster::Error, NoError = 0
    static constexpr uint8_t FLAG_SUCCESS = 1, FLAG_REINIT = 2;

    struct __attribute__((packed)) Record {
        uint32_t timeUs, seq;
        uint8_t result, flags;
        uint16_t pendingFailures;
        float payload[4];  // enc1.pos, enc2.pos, enc1.speed, enc2.speed as read
        float heldX, heldY;  // LocalizationPico::getX/getY after this attempt
    };
    struct __attribute__((packed)) Summary {
        uint32_t timeUs, lastSeq;
        uint16_t results[RESULT_COUNT];  // attempts per result since the previous summary
        uint32_t lostFailures, decimatedSuccesses, fifoDrops;
        uint32_t msSinceSuccess, msSinceAttempt;  // UINT32_MAX: never
        uint16_t busReinits, reserved;
    };
    static_assert(sizeof(Record) == 36 && sizeof(Summary) == 48);

    void attempt(uint32_t timeUs, uint32_t timeMs, uint8_t result, const float payload[4], float heldX, float heldY) {
        Record r{timeUs, ++seq, result, 0, static_cast<uint16_t>(failureCount), {}, heldX, heldY};
        std::memcpy(r.payload, payload, sizeof(r.payload));
        if (result == 0) r.flags |= FLAG_SUCCESS;
        if (reinitPending) r.flags |= FLAG_REINIT;
        reinitPending = false;
        if (result >= RESULT_COUNT) result = RESULT_COUNT - 1;
        if (results[result] != std::numeric_limits<uint16_t>::max()) ++results[result];
        lastAttemptMs = timeMs;
        everAttempted = true;
        if (result == 0) {
            lastSuccessMs = timeMs;
            everSucceeded = true;
            if (haveSuccess) saturatingIncrement(decimatedSuccesses);
            latestSuccess = r;
            haveSuccess = true;
        } else if (failureCount < FAILURE_CAPACITY) {
            failures[(failureHead + failureCount++) % FAILURE_CAPACITY] = r;
        } else {
            saturatingIncrement(lostFailures);
        }
    }
    bool enabled = true;  // hosted fixtures opt in; firmware always sends
    // A successful read within maxAgeMs of nowMs (same millisecond clock, wrap-safe).
    bool succeededWithin(uint32_t nowMs, uint32_t maxAgeMs) const {
        return everSucceeded && nowMs - lastSuccessMs <= maxAgeMs;
    }

    void noteReinit() {
        reinitPending = true;
        if (busReinits != std::numeric_limits<uint16_t>::max()) ++busReinits;
    }

    // Sink: UARTCommunication (mcbFreeSlots, mcbLog, droppedData, getCurrentTime).
    // Order: summary, failures, then one success per period with a slot left spare.
    template <class Sink> void drain(Sink& sink, uint32_t timeUs) {
        if (!enabled) return;
        const uint32_t now = sink.getCurrentTime();
        if (!summaryStarted) {
            summaryStarted = true;
            nextSummaryMs = now + SUMMARY_PERIOD_MS;
        }
        if (static_cast<int32_t>(now - nextSummaryMs) >= 0 && sink.mcbFreeSlots() > 0) {
            Summary s{timeUs, seq, {}, lostFailures, decimatedSuccesses, sink.droppedData(),
                      everSucceeded ? now - lastSuccessMs : UINT32_MAX,
                      everAttempted ? now - lastAttemptMs : UINT32_MAX, busReinits, 0};
            std::memcpy(s.results, results.data(), sizeof(s.results));
            if (sink.mcbLog(SUMMARY_CHANNEL, s)) {
                results.fill(0);
                nextSummaryMs += ((now - nextSummaryMs) / SUMMARY_PERIOD_MS + 1) * SUMMARY_PERIOD_MS;
            }
        }
        while (failureCount && sink.mcbFreeSlots() > 0 && sink.mcbLog(RECORD_CHANNEL, failures[failureHead])) {
            failureHead = (failureHead + 1) % FAILURE_CAPACITY;
            --failureCount;
        }
        if (haveSuccess && !failureCount && sink.mcbFreeSlots() > 1 &&
            (!successLogged || static_cast<int32_t>(now - lastSuccessLogMs) >= static_cast<int32_t>(SUCCESS_PERIOD_MS)) &&
            sink.mcbLog(RECORD_CHANNEL, latestSuccess)) {
            haveSuccess = false;
            successLogged = true;
            lastSuccessLogMs = now;
        }
    }

private:
    static void saturatingIncrement(uint32_t& value) {
        if (value != std::numeric_limits<uint32_t>::max()) ++value;
    }
    std::array<Record, FAILURE_CAPACITY> failures{};
    size_t failureHead = 0, failureCount = 0;
    Record latestSuccess{};
    std::array<uint16_t, RESULT_COUNT> results{};
    uint32_t seq = 0, lostFailures = 0, decimatedSuccesses = 0;
    uint32_t lastSuccessMs = 0, lastAttemptMs = 0, lastSuccessLogMs = 0, nextSummaryMs = 0;
    uint16_t busReinits = 0;
    bool haveSuccess = false, successLogged = false, everSucceeded = false, everAttempted = false;
    bool reinitPending = false, summaryStarted = false;
};
}  // namespace communication
