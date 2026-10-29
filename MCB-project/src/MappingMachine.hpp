// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include <cstdint>

namespace src {

class MappingMachine {
public:
    enum class State { Main, Setup, Ready, Mapping, Done };
    static constexpr uint32_t RETRY_MS = 500;
    static constexpr uint32_t BEEP_PERIOD_MS = 1000;
    static constexpr uint32_t BEEP_LENGTH_MS = 100;

    State getState() const { return state; }
    bool isActive() const { return state != State::Main; }
    bool canDrive() const { return state == State::Ready || state == State::Mapping; }
    bool readyBeep(uint32_t now) const {
        return state == State::Ready && (now - readySince) % BEEP_PERIOD_MS < BEEP_LENGTH_MS;
    }

    void requestStart() {
        if (state != State::Main) return;
        echoes[0] = 0;
        state = State::Setup;
        odd[0].awaiting = true;
    }

    void requestSave() {
        if (!canDrive()) return;
        if (state == State::Ready) state = State::Done;
        // Save incomplete is a send-only transition, immediately back to Mapping.
        ++queuedSaves;
    }

    void receivePing(uint8_t number, uint32_t now) {
        nowMs = now;
        if (number == 0) {
            if (state == State::Main) ++echoes[0];
        } else if (number == 2 || number == 4) {
            ++echoes[number];
            if (number == 2 && (state == State::Setup || state == State::Mapping)) {
                state = State::Ready;
                readySince = now;
            } else if (number == 4 && state == State::Ready) {
                state = State::Mapping;
            }
        } else if (number == 1 || number == 3) {
            auto& request = odd[number / 2];
            if (request.awaiting && request.sentForRequest) {
                request.awaiting = false;
                ++echoes[number];
            }
        }
    }

    // send(number) returns true only when the UART accepted the frame.
    template<typename Send>
    void tick(uint32_t now, Send send) {
        nowMs = now;
        auto& save = odd[1];
        if (queuedSaves != 0 && !save.awaiting && echoes[3] == 0 &&
            (!save.sent || now - save.lastSend >= RETRY_MS)) {
            --queuedSaves;
            save.awaiting = true;
            save.sentForRequest = false;
        }
        for (uint8_t number = 0; number < echoes.size(); ++number) {
            if (echoes[number] == 0) continue;
            if (!send(number)) return;
            --echoes[number];
            if (number == 1 || number == 3) recordSend(odd[number / 2], now);
            return;
        }
        for (uint8_t i = 0; i < odd.size(); ++i) {
            auto& request = odd[i];
            if (!request.awaiting || (request.sent && now - request.lastSend < RETRY_MS)) continue;
            if (!send(static_cast<uint8_t>(2 * i + 1))) return;
            recordSend(request, now);
            request.sentForRequest = true;
            return;
        }
    }

    bool hasPendingPing() const {
        for (auto count : echoes) if (count != 0) return true;
        for (const auto& request : odd) {
            if (request.awaiting && (!request.sent || nowMs - request.lastSend >= RETRY_MS)) return true;
        }
        return false;
    }

private:
    struct OddRequest {
        bool awaiting = false;
        bool sent = false;
        bool sentForRequest = false;
        uint32_t lastSend = 0;
    };
    static void recordSend(OddRequest& request, uint32_t now) {
        request.sent = true;
        request.lastSend = now;
    }
    State state = State::Main;
    uint32_t nowMs = 0;
    uint32_t readySince = 0;
    uint32_t queuedSaves = 0;
    std::array<uint32_t, 5> echoes{};
    std::array<OddRequest, 2> odd{};
};
}  // namespace src
