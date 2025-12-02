#pragma once
#include "tap/algorithms/smooth_pid.hpp"

constexpr static int INDEXER_MOTOR_MAX_SPEED = 18000; //6177; // With the 2006, this should give


constexpr static float AUTO_UNJAM_BALLS_PER_SEC_THRESH = 0.1;  // balls/sec, if the motor gets stopped or slowed enough to be below this speed, the index will consider unjamming
constexpr static float AUTO_UNJAM_TIME_UNDER_THRESH = 0.5;      // sec, if the motor gets stopped or slowed enough for this time, the index will start to unjam
constexpr static float AUTO_UNJAM_TIME_UNJAMMING = 0.1;         // sec, once unjamming, it will continue unjamming for this long

constexpr static float HOMING_TIMEOUT = 0.5;  // seconds, if we are spinning without finding a shot for this long, give up


#if defined(HERO)
// motor ratio * ball rolling * diameter or spacing of balls / wheel circumference
constexpr static int NUM_CHAMBERS = 0; //hero is weird, it indexes differently
constexpr static float GEAR_RATIO = 36.0f; //need to check if it is 36 exactly or 36ish
constexpr static float REV_PER_BALL = 42.0f / (50.0f * 3.1415926353f) * 2.0f * 36.0f; // GUESS, this value should be tuned since it isn't exactly known
constexpr static float REV_PER_BALL_BOTTOM = GEAR_RATIO; // revolutions per ball = gearbox * external reduction / chambers
constexpr static float LOAD_BALL_PER_SECOND = 5.0f;
constexpr static float UNJAM_BALL_PER_SECOND = -8.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};
constexpr static float INITIAL_BURST_NUM_BALLS = 0;
#elif defined(SENTRY)
constexpr static int NUM_CHAMBERS = 7;
constexpr static float GEAR_RATIO = 36.0f; //need to check if it is 36 exactly or 36ish
constexpr static float REV_PER_BALL = GEAR_RATIO / NUM_CHAMBERS; // revolutions per ball = ratio / chambers
constexpr static float UNJAM_BALL_PER_SECOND = -1.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};
constexpr static float INITIAL_BURST_NUM_BALLS = 0.9;
#elif defined(INFANTRY)
constexpr static int NUM_CHAMBERS = 8;
constexpr static float GEAR_RATIO = 36.0f; //need to check if it is 36 exactly or 36ish
constexpr static float REV_PER_BALL = GEAR_RATIO / NUM_CHAMBERS; // revolutions per ball = ratio / chambers
constexpr static float UNJAM_BALL_PER_SECOND = -1.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};
constexpr static float INITIAL_BURST_NUM_BALLS = 0.9;
constexpr static float INITIAL_INDEX_OFFSET = 0.93f;
#else
constexpr static int NUM_CHAMBERS = 7;
constexpr static float GEAR_RATIO = 36.0f; //need to check if it is 36 exactly or 36ish
constexpr static float REV_PER_BALL = GEAR_RATIO / NUM_CHAMBERS; // revolutions per ball = ratio / chambers
constexpr static float UNJAM_BALL_PER_SECOND = -1.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};
constexpr static float INITIAL_BURST_NUM_BALLS = 0.9;
#endif

#ifndef HERO
constexpr static float REV_PER_BALL_BOTTOM = 0; //doesn't apply to non heros
constexpr static float LOAD_BALL_PER_SECOND = 0; //doesn't apply to non heros
#endif