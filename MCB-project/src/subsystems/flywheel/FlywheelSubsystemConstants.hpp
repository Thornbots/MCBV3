#pragma once
#include "tap/algorithms/smooth_pid.hpp"

#if defined(HERO)
constexpr static int FLYWHEEL_MOTOR_MAX_RPM = 8333;  // We had 5000 last year, and we can go 30/18 times as fast. So 5000 * 30/18
constexpr static int FLYWHEEL_RADIUS_MM = 60;
#elif defined(SENTRY)
constexpr static int FLYWHEEL_MOTOR_MAX_RPM = 7500;  // We had 5000 last year, and we can go 30/18 times as fast. So 5000 * 30/18
constexpr static int FLYWHEEL_RADIUS_MM = 60;
#elif defined(INFANTRY)
constexpr static int FLYWHEEL_MOTOR_MAX_RPM = 6100;  // We had 5000 last year, and we can go 30/18 times as fast. So 5000 * 30/18
constexpr static int FLYWHEEL_RADIUS_MM = 60;
#else
constexpr static int FLYWHEEL_MOTOR_MAX_RPM = 8333;  // We had 5000 last year, and we can go 30/18 times as fast. So 5000 * 30/18
constexpr static int FLYWHEEL_RADIUS_MM = 40;
#endif

constexpr static tap::algorithms::SmoothPidConfig PID_CONF_FLYWHEEL = {40, 0.1, 0, 10.0, 10000, 1, 0, 1, 0, 50, 0};