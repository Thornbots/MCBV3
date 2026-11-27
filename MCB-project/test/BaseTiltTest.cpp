#include <gtest/gtest.h>
#include <cmath>
#include <util/BaseTilt.hpp>

namespace {
constexpr float kForward[3] = {1, 0, 0};
constexpr float kDt = 0.001f;
}

// A 3 deg nose-down pitch (IMU +y) reads +3 deg in a down-positive encoder sense.
TEST(BaseTilt, TracksFastPitchInEncoderSense) {
    BaseTilt tilt(kForward, -1.0f, 2.0f);
    const float rate = 0.0523599f / 0.05f;
    for (int i = 0; i < 50; ++i) tilt.update(0, rate, 0, kDt);
    EXPECT_NEAR(tilt.angle, 0.0523599f, 0.002f);
    EXPECT_GT(tilt.rate, 0.9f * rate);
}

// Roll about the barrel axis, or yaw while level, does not tilt the barrel.
TEST(BaseTilt, IgnoresRollAndLevelYaw) {
    BaseTilt roll(kForward, -1.0f, 2.0f), yaw(kForward, -1.0f, 2.0f);
    for (int i = 0; i < 50; ++i) {
        roll.update(1.0f, 0, 0, kDt);
        yaw.update(0, 0, 10.0f, kDt);
    }
    EXPECT_NEAR(roll.angle, 0.0f, 1e-4f);
    EXPECT_NEAR(yaw.angle, 0.0f, 1e-4f);
}

// A held tilt or constant gyro bias decays into the encoder frame.
TEST(BaseTilt, HighPassesHeldTiltAndBias) {
    BaseTilt tilt(kForward, -1.0f, 2.0f);
    for (int i = 0; i < 50; ++i) tilt.update(0, 1.0f, 0, kDt);
    for (int i = 0; i < 20000; ++i) tilt.update(0, 0.001f, 0, kDt);
    EXPECT_NEAR(tilt.angle, 0.002f, 0.002f);
}
