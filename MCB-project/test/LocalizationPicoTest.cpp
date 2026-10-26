// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include <cstring>
#include "communication/LocalizationPico.hpp"
#include "hosted/clock.hpp"

namespace {
struct PicoMaster {
    inline static bool needsReinit = false, accept = true;
    inline static modm::I2cTransaction* active = nullptr;
    inline static unsigned starts = 0;
    static modm::I2cMaster::Error getErrorState() { return modm::I2cMaster::Error::SoftwareReset; }
    static bool start(modm::I2cTransaction* transaction, modm::I2c::ConfigurationHandler) {
        ++starts;
        if (!accept || !transaction->attaching()) return false;
        active = transaction;
        return true;
    }
    static void reset() {
        if (active) active->detaching(modm::I2c::DetachCause::ErrorCondition);
        active = nullptr;
        needsReinit = false;
    }
    static void complete() {
        const float values[4] = {1, 2, 3, 4};
        auto reading = active->reading();
        std::memcpy(reading.buffer, values, sizeof(values));
        active->detaching(modm::I2c::DetachCause::NormalStop);
        active = nullptr;
    }
};
class PicoPollTest : public ::testing::Test {
protected:
    communication::LocalizationPico<PicoMaster> pico;
    void SetUp() override {
        hosted::timeMs = 0;
        PicoMaster::active = nullptr;
        PicoMaster::needsReinit = false;
        PicoMaster::accept = true;
        PicoMaster::starts = 0;
    }
};
TEST_F(PicoPollTest, HungFirstReadRequestsRecoveryAndCanSucceedAfterReset) {
    pico.run();
    ASSERT_NE(PicoMaster::active, nullptr);
    hosted::timeMs = 9;
    pico.run();
    EXPECT_FALSE(PicoMaster::needsReinit);
    hosted::timeMs = 10;
    pico.run();
    ASSERT_TRUE(PicoMaster::needsReinit);
    EXPECT_FALSE(pico.isFresh(10, 50));
    PicoMaster::reset();  // I2CCommunication::refresh does this before polling again.
    pico.run();
    ASSERT_NE(PicoMaster::active, nullptr);
    PicoMaster::complete();
    pico.run();
    EXPECT_TRUE(pico.isFresh(10, 50));
    EXPECT_FLOAT_EQ(pico.getY(), 2);
    EXPECT_TRUE(pico.isFresh(59, 50));
    EXPECT_FALSE(pico.isFresh(60, 50));
    PicoMaster::reset();
}
TEST_F(PicoPollTest, StartupRecoveryOrphanDoesNotPermanentlyBlockPico) {
    pico.run();
    ASSERT_NE(PicoMaster::active, nullptr);
    // Old STM32 recoverStuckBus discarded this pointer without detaching it.
    PicoMaster::active = nullptr;
    PicoMaster::reset();
    hosted::timeMs = 1;
    pico.run();
    EXPECT_EQ(PicoMaster::active, nullptr);
    hosted::timeMs = 10;
    pico.run();
    ASSERT_TRUE(PicoMaster::needsReinit);
    PicoMaster::reset();
    pico.run();
    ASSERT_NE(PicoMaster::active, nullptr);
    PicoMaster::complete();
    pico.run();
    EXPECT_TRUE(pico.isFresh(10, 50));
    PicoMaster::reset();
}
TEST_F(PicoPollTest, FirstReadCannotAttachRequestsRecovery) {
    PicoMaster::accept = false;
    pico.run();
    hosted::timeMs = 10;
    pico.run();
    EXPECT_TRUE(PicoMaster::needsReinit);
    EXPECT_FALSE(pico.isFresh(10, 50));
    PicoMaster::reset();
    PicoMaster::accept = true;
    pico.run();
    EXPECT_NE(PicoMaster::active, nullptr);
    PicoMaster::reset();
}
TEST_F(PicoPollTest, FailedFirstReadDoesNotLatchTransactionInError) {
    pico.run();
    ASSERT_NE(PicoMaster::active, nullptr);
    PicoMaster::active->detaching(modm::I2c::DetachCause::ErrorCondition);
    PicoMaster::active = nullptr;
    pico.run();
    ASSERT_NE(PicoMaster::active, nullptr);
    PicoMaster::complete();
    pico.run();
    EXPECT_TRUE(pico.isFresh(0, 50));
    PicoMaster::reset();
}
TEST_F(PicoPollTest, TimeoutAndFreshnessSurviveClockRollover) {
    hosted::timeMs = UINT32_MAX - 4;
    pico.run();
    hosted::timeMs = 5;
    pico.run();
    EXPECT_TRUE(PicoMaster::needsReinit);
    PicoMaster::reset();
}
}
