// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include <cstring>
#include "communication/LocalizationPico.hpp"
#include "hosted/clock.hpp"
#include "subsystems/jetson/JetsonSubsystem.hpp"

namespace {
struct PicoMaster {
    inline static modm::I2cTransaction* active = nullptr;
    static bool start(modm::I2cTransaction* transaction, modm::I2c::ConfigurationHandler) {
        if (!transaction->attaching()) return false;
        active = transaction;
        return true;
    }
    static void complete() {
        const float values[4] = {1, 2, 3, 4};
        auto reading = active->reading();
        std::memcpy(reading.buffer, values, sizeof(values));
        active->detaching(modm::I2c::DetachCause::NormalStop);
        active = nullptr;
    }
};
class PicoFreshnessTest : public ::testing::Test {
protected:
    communication::LocalizationPico<PicoMaster> pico;
    void SetUp() override { hosted::timeMs = 0; PicoMaster::active = nullptr; }
    void TearDown() override { PicoMaster::active = nullptr; }
    void succeedAt(uint32_t now) {
        hosted::timeMs = now;
        pico.run();
        ASSERT_NE(PicoMaster::active, nullptr);
        PicoMaster::complete();
        pico.run();
    }
    subsystems::OdomStatus statusAt(uint32_t now) {
        return pico.isFresh(now, STALE_MS) ? subsystems::ODOM_PODS : subsystems::ODOM_I2C_DEAD;
    }
    static constexpr uint32_t STALE_MS = 500;  // JetsonSubsystem::POD_STALE_MS
};
TEST_F(PicoFreshnessTest, NoSuccessReportsDeadIncludingHungFirstRead) {
    EXPECT_EQ(statusAt(0), 2);
    pico.run();
    hosted::timeMs = 1000;
    pico.run();
    EXPECT_EQ(statusAt(1000), 2);
}
TEST_F(PicoFreshnessTest, SuccessIsHealthyUntilExactlyFiveHundredMilliseconds) {
    succeedAt(100);
    EXPECT_FLOAT_EQ(pico.getY(), 2);
    EXPECT_EQ(statusAt(100), 0);
    EXPECT_EQ(statusAt(599), 0);
    hosted::timeMs = 600;
    pico.run(); // The next read remains hung; polling must not refresh freshness.
    EXPECT_EQ(statusAt(600), 2);
    EXPECT_EQ(statusAt(601), 2);
}
TEST_F(PicoFreshnessTest, MeasuredSentryCadenceStaysHealthy) {
    for (uint32_t t = 100; t <= 100 + 10 * 260; t += 260) {
        succeedAt(t);
        EXPECT_EQ(statusAt(t + 259), 0);
    }
}
TEST_F(PicoFreshnessTest, LaterSuccessRefreshesAge) {
    succeedAt(100);
    succeedAt(400);
    EXPECT_EQ(statusAt(899), 0);
    EXPECT_EQ(statusAt(900), 2);
}
TEST_F(PicoFreshnessTest, FreshnessSurvivesUnsignedClockRollover) {
    succeedAt(UINT32_MAX - 249);
    EXPECT_EQ(statusAt(249), 0);
    EXPECT_EQ(statusAt(250), 2);
}
}
