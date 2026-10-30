// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#ifdef MCB_HOSTED
#include <cstdint>

namespace hosted {
// Private SOCK_SEQPACKET fixture interface; never a robot UART message.
struct MappingInput {
    uint32_t action; // 0 sticks only, 1 start, 3 save
    float right, forward, spin;
};
struct MappingStatus {
    uint32_t state, timeMs, transitions, failedWrites, readyBeep;
    float loopUs, periodUs, headForwardError;
};
static_assert(sizeof(MappingInput) == 16 && sizeof(MappingStatus) == 32);
}
#endif
