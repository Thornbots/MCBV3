// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#ifdef MCB_CAN_BUS_HEALTH
#include "drivers.hpp"
#include "tap/architecture/clock.hpp"

namespace src {
inline void sampleCanBusHealth(Drivers& drivers) {
    drivers.canHealth.sample(drivers.can.isReadyToSend(tap::can::CanBus::CAN_BUS1),
                             drivers.can.isReadyToSend(tap::can::CanBus::CAN_BUS2));
}

template <typename Jetson>
void reportCanBusHealth(Drivers& drivers, Jetson& jetson) {
    const uint32_t now = tap::arch::clock::getTimeMilliseconds();
    if (!drivers.canHealth.due(now)) return;
    std::array<uint32_t, 2> esr{};
    bool valid = false;
#if !defined(PLATFORM_HOSTED) && defined(CAN1) && defined(CAN2)
    esr = {CAN1->ESR, CAN2->ESR};
    valid = true;
#endif
    const auto payload = drivers.canHealth.finish(now, esr, valid);
    // Drop this complete window if another DEBUG producer owns the slot; never retry.
    jetson.sendDebug(payload.data(), payload.size());
}
} // namespace src
#endif
