#pragma once
#include "tap/algorithms/smooth_pid.hpp"

// START getters and setters
#if defined(HERO)
constexpr tap::algorithms::SmoothPidConfig drivetrainPIDConfig{1.5, 0, -0.3, 0, 2.0, 1.0f, 0.0f, 1.0f, 0.0f, 0.05};

constexpr float ERR_DEADZONE_ROT = 0.05;  // error deadzone for drivetrain rad/s

constexpr float PEEK_LEFT_AMT = 0.6;    // amount to peek left
constexpr float PEEK_RIGHT_AMT = -0.6;  // amount to peek right
#elif defined(SENTRY)
constexpr tap::algorithms::SmoothPidConfig drivetrainPIDConfig{6.0, 0, 0, 0, 2.0, 1.0f, 0.0f, 1.0f, 0.0f, 0.05};

constexpr float ERR_DEADZONE_ROT = 0.05;  // error deadzone for drivetrain rad/s

constexpr float PEEK_LEFT_AMT = 0.6;    // amount to peek left
constexpr float PEEK_RIGHT_AMT = -0.6;  // amount to peek right
#elif defined(INFANTRY)
constexpr tap::algorithms::SmoothPidConfig drivetrainPIDConfig{6.0, 0, -0, 0, 6.0, 1.0f, 0.0f, 1.0f, 0.0f, 0.05};

constexpr float PEEK_LEFT_AMT = -0.45;    // amount to peek left
constexpr float PEEK_RIGHT_AMT = 0.45;  // amount to peek right

#else
constexpr tap::algorithms::SmoothPidConfig drivetrainPIDConfig{1.5, 0, -0.3, 0, 2.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.05};

constexpr float PEEK_LEFT_AMT = 0.6;    // amount to peek left
constexpr float PEEK_RIGHT_AMT = -0.6;  // amount to peek right
#endif

// after the ifdefs

constexpr float DEFAULT_POWER_LIMIT = 45;  // default power limit for drivetrain
