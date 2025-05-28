#pragma once
#include "tap/algorithms/smooth_pid.hpp"

constexpr static int INDEXER_MOTOR_MAX_SPEED = 2000; //6177; // With the 2006, this should give

#if defined(HERO)
// motor ratio * ball rolling * diameter or spacing of balls / wheel circumference
constexpr static float REV_PER_BALL = 42.0f / (50.0f * 3.1415926353f) * 2.0f * 36.0f; // GUESS, this value should be tuned since it isn't exactly known
constexpr static float REV_PER_BALL_BOTTOM = 36.0f * 1.0f / 5.0f; // revolutions per ball = gearbox * external reduction / chambers
constexpr static float UNJAM_BALL_PER_SECOND = -1.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};
#elif defined(SENTRY)
constexpr static float REV_PER_BALL = 36.0f / 7.0f; // revolutions per ball = ratio / chambers
constexpr static float REV_PER_BALL_BOTTOM = 0; //doesn't apply
constexpr static float UNJAM_BALL_PER_SECOND = -1.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};
#elif defined(INFANTRY)
constexpr static float REV_PER_BALL = 36.0f / 8.0f; // revolutions per ball = ratio / chambers
constexpr static float REV_PER_BALL_BOTTOM = 0; //doesn't apply
constexpr static float UNJAM_BALL_PER_SECOND = -1.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};
#else
constexpr static float REV_PER_BALL = 36.0f / 7.0f; // revolutions per ball = ratio / chambers
constexpr static float REV_PER_BALL_BOTTOM = 0; //doesn't apply
constexpr static float UNJAM_BALL_PER_SECOND = -1.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};
#endif  
