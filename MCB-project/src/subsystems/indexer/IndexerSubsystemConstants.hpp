#pragma once
#include "tap/algorithms/smooth_pid.hpp"

constexpr static int INDEXER_MOTOR_MAX_SPEED = 18000; //6177; // With the 2006, this should give


//only hero has unjam code for now
constexpr static float AUTO_UNJAM_BALLS_PER_SEC_THRESH = 0.1;  // balls/sec, if the motor gets stopped or slowed enough to be below this speed, the index will consider unjamming
constexpr static float AUTO_UNJAM_TIME_UNDER_THRESH = 0.1;      // sec, if the motor gets stopped or slowed enough for this time, the index will start to unjam
constexpr static float AUTO_UNJAM_TIME_UNJAMMING = 0.1;         // sec, once unjamming, it will continue unjamming for this long


#if defined(HERO)
// motor ratio * ball rolling * diameter or spacing of balls / wheel circumference
constexpr static float REV_PER_BALL = 42.0f / (50.0f * 3.1415926353f) * 2.0f * 36.0f; // GUESS, this value should be tuned since it isn't exactly known
constexpr static float REV_PER_BALL_BOTTOM = 36.0f; // revolutions per ball = gearbox * external reduction / chambers
constexpr static float LOAD_BALL_PER_SECOND = 5.0f;
constexpr static float UNJAM_BALL_PER_SECOND = -5.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};
constexpr static float INITIAL_BURST_NUM_BALLS = 0;
#elif defined(SENTRY)
constexpr static float REV_PER_BALL = 36.0f / 7.0f; // revolutions per ball = ratio / chambers
constexpr static float UNJAM_BALL_PER_SECOND = -1.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};
constexpr static float INITIAL_BURST_NUM_BALLS = 0.9;
#elif defined(INFANTRY)
constexpr static float REV_PER_BALL = 36.0f / 8.0f; // revolutions per ball = ratio / chambers
constexpr static float UNJAM_BALL_PER_SECOND = -1.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};
constexpr static float INITIAL_BURST_NUM_BALLS = 0.9;
#else
constexpr static float REV_PER_BALL = 36.0f / 7.0f; // revolutions per ball = ratio / chambers
constexpr static float UNJAM_BALL_PER_SECOND = -1.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};
constexpr static float INITIAL_BURST_NUM_BALLS = 0.9;
#endif  

#ifndef HERO
constexpr static float REV_PER_BALL_BOTTOM = 0; //doesn't apply to non heros
constexpr static float LOAD_BALL_PER_SECOND = 0; //doesn't apply to non heros
#endif