#pragma once
#include "tap/algorithms/smooth_pid.hpp"

// START getters and setters
#if defined(HERO)
static constexpr tap::algorithms::SmoothPidConfig drivetrainPIDConfig{8, 0, -0, 0, 10.0, 1.0f, 0.0f, 1.0f, 0.0f, 0.05};

static constexpr uint16_t INITIAL_POWER_LIMIT_3V3 = 60;  // W

static constexpr float PEEK_LEFT_AMT = -PI/2;    // amount to peek left
static constexpr float PEEK_RIGHT_AMT = PI/2;  // amount to peek right

static constexpr float MAX_LINEAR_SPEED = 4.3;
static constexpr int SPIN_DIRECTION = 1;

#elif defined(SENTRY)
static constexpr tap::algorithms::SmoothPidConfig drivetrainPIDConfig{6.0, 0, 0, 0, 2.0, 1.0f, 0.0f, 1.0f, 0.0f, 0.05};

static constexpr float ERR_DEADZONE_ROT = 0.05;  // error deadzone for drivetrain rad/s

static constexpr uint16_t INITIAL_POWER_LIMIT_3V3 = 100;  // W

static constexpr float PEEK_LEFT_AMT = -0.45;    // amount to peek left
static constexpr float PEEK_RIGHT_AMT = 0.45;  // amount to peek right

static constexpr float MAX_LINEAR_SPEED = 2.5;
static constexpr int SPIN_DIRECTION = 1;

#elif defined(INFANTRY)
static constexpr tap::algorithms::SmoothPidConfig drivetrainPIDConfig{8, 0, -0, 0, 10.0, 1.0f, 0.0f, 1.0f, 0.0f, 0.05};

static constexpr uint16_t INITIAL_POWER_LIMIT_3V3 = 45;  // W

static constexpr float PEEK_LEFT_AMT = -0.45;    // amount to peek left
static constexpr float PEEK_RIGHT_AMT = 0.45;  // amount to peek right

static constexpr float MAX_LINEAR_SPEED = 4.3;
static constexpr int SPIN_DIRECTION = 1;

#else
static constexpr tap::algorithms::SmoothPidConfig drivetrainPIDConfig{1.5, 0, -0.3, 0, 2.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.05};

static constexpr uint16_t INITIAL_POWER_LIMIT_3V3 = 45;  // W

static constexpr float PEEK_LEFT_AMT = -0.45;    // amount to peek left
static constexpr float PEEK_RIGHT_AMT = 0.45;  // amount to peek right

static constexpr float MAX_LINEAR_SPEED = 2.5;
static constexpr int SPIN_DIRECTION = 1;

#endif



// after the ifdefs


static constexpr float DEFAULT_POWER_LIMIT = 45;  // default power limit for drivetrain
