// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include "util/CanBusHealth.hpp"

namespace {
uint32_t read32(const src::CanBusHealth::Payload& payload, size_t offset) {
    uint32_t value = 0;
    for (unsigned i = 0; i < 4; ++i)
        value |= static_cast<uint32_t>(payload[offset + i]) << (8 * i);
    return value;
}
}

TEST(CanBusHealthTest, EncodesIndependentBusesAndAllMotorIdsLittleEndian) {
    src::CanBusHealth health;
    health.start(123);
    health.sample(false, true);
    health.sample(false, false);
    for (size_t bus = 0; bus < 2; ++bus)
        for (uint32_t id = 0x201; id <= 0x208; ++id)
            for (unsigned i = 0; i < id - 0x200 + bus; ++i) health.feedback(bus, id);
    health.feedback(2, 0x201);
    health.feedback(0, 0x200);
    health.feedback(1, 0x209);
    EXPECT_FALSE(health.due(1122));
    EXPECT_TRUE(health.due(1123));
    const auto payload = health.finish(1123, {0x12345678, 0xabcdef01}, true);
    EXPECT_EQ(payload.size(), 103U);
    EXPECT_EQ(payload[0], 1);
    EXPECT_EQ(read32(payload, 1), 0U);
    EXPECT_EQ(read32(payload, 5), 1000U);
    EXPECT_EQ(read32(payload, 9), 2U);
    EXPECT_EQ(read32(payload, 13), 2U);
    EXPECT_EQ(read32(payload, 58), 1U);
    for (size_t bus = 0; bus < 2; ++bus) {
        EXPECT_EQ(read32(payload, 17 + 45 * bus), 0U);
        for (size_t id = 0; id < 8; ++id)
            EXPECT_EQ(read32(payload, 21 + 45 * bus + 4 * id), id + 1 + bus);
        EXPECT_EQ(payload[57 + 45 * bus], 1);
    }
    EXPECT_EQ(read32(payload, 53), 0x12345678U);
    EXPECT_EQ(payload[53], 0x78);
    EXPECT_EQ(payload[56], 0x12);
    EXPECT_EQ(read32(payload, 98), 0xabcdef01U);
}

TEST(CanBusHealthTest, WindowsResetEvenWhenDroppedAndClockWraps) {
    src::CanBusHealth health;
    health.start(UINT32_MAX - 499);
    health.sample(false, false);
    health.feedback(0, 0x207);
    EXPECT_FALSE(health.due(499));
    EXPECT_TRUE(health.due(500));
    health.finish(500, {}, false); // Sender may discard the result.
    const auto next = health.finish(1750, {}, false);
    EXPECT_EQ(read32(next, 1), 1U);
    EXPECT_EQ(read32(next, 5), 1250U);
    EXPECT_EQ(read32(next, 9), 0U);
    for (size_t offset = 13; offset < next.size(); ++offset) EXPECT_EQ(next[offset], 0);
}
