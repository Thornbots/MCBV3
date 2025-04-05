#pragma once
/*
Constants file for all robots subsystem constants, copied from gimbal
*/
constexpr static float PI_CONST = 3.14159;
constexpr static int ODOMETRY_MOTOR_MAX_VOLTAGE = 24000;  // Should be the voltage of the battery. Unless the motor maxes out below that.
static constexpr float dt = 0.002f;

#if defined(HERO)
constexpr static int ODOMETRY_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                      // //TODO: Check the datasheets

static constexpr float ODOMETRY_OFFSET = 3 * PI_CONST / 4;

static constexpr float ODOMETRY_TOTAL_RATIO = 32319.0f / 748.0f;  // unitless, ratio of encoder counts to degrees of rotation

static constexpr int ODOMETRY_DIST_RANGE = 18000;

#elif defined(SENTRY)
constexpr static int ODOMETRY_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                      // //TODO: Check the datasheets

static constexpr float ODOMETRY_OFFSET = 0;

static constexpr float ODOMETRY_TOTAL_RATIO = 1.0f;  // unitless, ratio of encoder counts to degrees of rotation

static constexpr int ODOMETRY_DIST_RANGE = 18000;

#elif defined(INFANTRY)
constexpr static int ODOMETRY_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                      // //TODO: Check the datasheets

static constexpr float ODOMETRY_OFFSET = 0;

static constexpr float ODOMETRY_TOTAL_RATIO = 32319.0f / 748.0f;  // unitless, ratio of encoder counts to degrees of rotation

static constexpr int ODOMETRY_DIST_RANGE = 16384 / 4;  // 20/4 = 5 amps

#else
constexpr static int ODOMETRY_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                      // //TODO: Check the datasheets

static constexpr float ODOMETRY_OFFSET = 3 * PI_CONST / 4;

static constexpr float ODOMETRY_TOTAL_RATIO = 1.0f;  // unitless, ratio of encoder counts to degrees of rotation

static constexpr int ODOMETRY_DIST_RANGE = 18000;

#endif