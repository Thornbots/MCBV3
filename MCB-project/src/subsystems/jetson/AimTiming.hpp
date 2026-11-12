#pragma once

#include <cstdint>

namespace aim {
// Opt-in DWT evidence, read through a debugger with the normal scheduler running.
struct CycleTiming {
    uint32_t calls = 0, maximum = 0, over_1ms = 0;
    void record(uint32_t cycles) {
        ++calls;
        if (cycles > maximum) maximum = cycles;
        if (cycles >= 168000) ++over_1ms;
    }
};
struct AimTiming {
    CycleTiming replan, control, gate, handoff, failed_plan;
};
inline AimTiming aim_timing;
inline uint32_t cycle_now() {
#if defined(MCB_AIM_TIMING) && defined(__arm__)
    return *reinterpret_cast<volatile uint32_t*>(0xE0001004);
#else
    return 0;
#endif
}
inline void enable_aim_timing() {
#if defined(MCB_AIM_TIMING) && defined(__arm__)
    *reinterpret_cast<volatile uint32_t*>(0xE000EDFC) |= 1U << 24;
    *reinterpret_cast<volatile uint32_t*>(0xE0001000) |= 1;
#endif
}
}  // namespace aim
