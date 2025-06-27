#pragma once
#include "tap/algorithms/smooth_pid.hpp"

#if defined(HERO)
constexpr static int FLYWHEEL_MOTOR_MAX_RPM = 5900;  
constexpr static int FLYWHEEL_RADIUS_MM = 60;
#elif defined(SENTRY)
constexpr static int FLYWHEEL_MOTOR_MAX_RPM = 7000;  
constexpr static int FLYWHEEL_RADIUS_MM = 48;
#elif defined(INFANTRY)
constexpr static int FLYWHEEL_MOTOR_MAX_RPM = 6100; 
constexpr static int FLYWHEEL_RADIUS_MM = 60;
#else
constexpr static int FLYWHEEL_MOTOR_MAX_RPM = 8333; 
constexpr static int FLYWHEEL_RADIUS_MM = 40;
#endif

constexpr static tap::algorithms::SmoothPidConfig PID_CONF_FLYWHEEL = {30, 0.1, 0, 10.0, 12000, 1, 0, 1, 0, 50, 0};