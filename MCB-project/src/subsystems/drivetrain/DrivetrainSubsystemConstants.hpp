#pragma once
#include "tap/algorithms/smooth_pid.hpp"

// START getters and setters
#if defined(HERO)
static constexpr tap::algorithms::SmoothPidConfig drivetrainPIDConfig{8, 0, -0, 0, 10.0, 1.0f, 0.0f, 1.0f, 0.0f, 0.05};

static constexpr uint16_t INITIAL_POWER_LIMIT_3V3 = 60;  // W

static constexpr float PEEK_LEFT_AMT = -PI/2;    // amount to peek left
static constexpr float PEEK_RIGHT_AMT = PI/2;  // amount to peek right

static constexpr float MAX_LINEAR_SPEED = 4.3;
static constexpr int SPIN_VELOCITY = 12.0f; //rad/s

static constexpr int MAX_LINEAR_VELOCITY_TIMES_100 = 175; //175 cm/s or 1.75 m/s
static constexpr int MIN_LINEAR_VELOCITY_TIMES_100 = 75;
static constexpr int LINEAR_VELOCITY_INCREMENT_TIMES_100 = 25;
static constexpr int MAX_NOSPIN_LINEAR_VELOCITY_TIMES_100 = 430;

#elif defined(SENTRY)
static constexpr tap::algorithms::SmoothPidConfig drivetrainPIDConfig{6.0, 0, 0, 0, 2.0, 1.0f, 0.0f, 1.0f, 0.0f, 0.05};

static constexpr float ERR_DEADZONE_ROT = 0.05;  // error deadzone for drivetrain rad/s

static constexpr uint16_t INITIAL_POWER_LIMIT_3V3 = 100;  // W

static constexpr float PEEK_LEFT_AMT = -0.45;    // amount to peek left
static constexpr float PEEK_RIGHT_AMT = 0.45;  // amount to peek right

static constexpr float MAX_LINEAR_SPEED = 5.25;
static constexpr int SPIN_VELOCITY = 12.0f; //rad/s, negative so sentry spins the other way than standard and hero

static constexpr int MAX_LINEAR_VELOCITY_TIMES_100 = 350;
static constexpr int MIN_LINEAR_VELOCITY_TIMES_100 = 100;
static constexpr int LINEAR_VELOCITY_INCREMENT_TIMES_100 = 50;
static constexpr int MAX_NOSPIN_LINEAR_VELOCITY_TIMES_100 = 1200;

#elif defined(INFANTRY)
static constexpr tap::algorithms::SmoothPidConfig drivetrainPIDConfig{8, 0, -0, 0, 10.0, 1.0f, 0.0f, 1.0f, 0.0f, 0.05};

static constexpr uint16_t INITIAL_POWER_LIMIT_3V3 = 45;  // W

static constexpr float PEEK_LEFT_AMT = -0.45;    // amount to peek left
static constexpr float PEEK_RIGHT_AMT = 0.45;  // amount to peek right

static constexpr float MAX_LINEAR_SPEED = 4.3;
static constexpr int SPIN_VELOCITY = 12.0f; //rad/s

static constexpr int MAX_LINEAR_VELOCITY_TIMES_100 = 175;
static constexpr int MIN_LINEAR_VELOCITY_TIMES_100 = 75;
static constexpr int LINEAR_VELOCITY_INCREMENT_TIMES_100 = 25;
static constexpr int MAX_NOSPIN_LINEAR_VELOCITY_TIMES_100 = 430;

#else
static constexpr tap::algorithms::SmoothPidConfig drivetrainPIDConfig{1.5, 0, -0.3, 0, 2.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.05};

static constexpr uint16_t INITIAL_POWER_LIMIT_3V3 = 45;  // W

static constexpr float PEEK_LEFT_AMT = -0.45;    // amount to peek left
static constexpr float PEEK_RIGHT_AMT = 0.45;  // amount to peek right

static constexpr float MAX_LINEAR_SPEED = 2.5;
static constexpr int SPIN_VELOCITY = 12.0f; //rad/s

static constexpr int MAX_LINEAR_VELOCITY_TIMES_100 = 175;
static constexpr int MIN_LINEAR_VELOCITY_TIMES_100 = 75;
static constexpr int LINEAR_VELOCITY_INCREMENT_TIMES_100 = 25;
static constexpr int MAX_NOSPIN_LINEAR_VELOCITY_TIMES_100 = 430;

#endif



// after the ifdefs


static constexpr float DEFAULT_POWER_LIMIT = 45;  // default power limit for drivetrain
