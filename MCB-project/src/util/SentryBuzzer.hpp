// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cstdint>
#include "tap/communication/sensors/buzzer/buzzer.hpp"

namespace src {
// Successful sentry pod reads arrive about every 260 ms.
inline constexpr uint32_t POD_STALE_MS = 500;

class SentryBuzzer {
public:
    static constexpr uint32_t BOOT_GRACE_MS = 3000;
    static constexpr uint32_t FAULT_PERIOD_MS = 1000;
    static constexpr uint32_t FAULT_BEEP_MS = 100;
    static constexpr uint32_t FAULT_GAP_MS = 100;
    static constexpr uint32_t FAULT_HZ = 2000;
    static constexpr uint32_t READY_HZ = 1000;
    static constexpr uint32_t HIT_HZ = 4000;

    explicit SentryBuzzer(tap::gpio::Pwm* pwm) : pwm(pwm) {}
    void update(uint32_t now, bool fresh) {
        if (!started) { bootSince = now; started = true; }
        if (now - bootSince >= BOOT_GRACE_MS) bootComplete = true;
        const bool nextFault = bootComplete && !fresh;
        if (nextFault && !fault) faultSince = now;
        fault = nextFault;
        nowMs = now;
        apply();
    }
    void setReady(bool on) { ready = on; apply(); }
    void setHit(bool on) { hit = on; apply(); }
    uint32_t frequency() const {
        if (fault) {
            const auto phase = (nowMs - faultSince) % FAULT_PERIOD_MS;
            return (phase < FAULT_BEEP_MS ||
                    (phase >= FAULT_BEEP_MS + FAULT_GAP_MS &&
                     phase < 2 * FAULT_BEEP_MS + FAULT_GAP_MS)) ? FAULT_HZ : 0;
        }
        if (ready) return READY_HZ;
        return hit ? HIT_HZ : 0;
    }
private:
    void apply() {
        const auto note = frequency();
        if (note) tap::buzzer::playNote(pwm, note);
        else tap::buzzer::silenceBuzzer(pwm);
    }
    tap::gpio::Pwm* pwm;
    uint32_t nowMs = 0, faultSince = 0, bootSince = 0;
    bool started = false, bootComplete = false, fault = false, ready = false, hit = false;
};
}  // namespace src
