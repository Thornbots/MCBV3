#pragma once
#include "tap/algorithms/smooth_pid.hpp"

constexpr static int INDEXER_MOTOR_MAX_SPEED = 18000; //6177; // With the 2006, this should give


constexpr static float AUTO_UNJAM_BALLS_PER_SEC_THRESH = 0.1;  // balls/sec, if the motor gets stopped or slowed enough to be below this speed, the index will consider unjamming
constexpr static float AUTO_UNJAM_TIME_UNDER_THRESH = 0.5;      // sec, if the motor gets stopped or slowed enough for this time, the index will start to unjam
constexpr static float AUTO_UNJAM_TIME_UNJAMMING = 0.1;         // sec, once unjamming, it will continue unjamming for this long

constexpr static float HOMING_TIMEOUT = 8;  // seconds, if we are spinning without finding a shot for this long, give up. If it takes this long, we probably don't have any projectiles and we wouldn't be shooting anyway
constexpr static float HOMING_BALLS_PER_SECOND = -0.25;  // balls per second, needs to be negative


#if defined(HERO)
// motor ratio * ball rolling * diameter or spacing of balls / wheel circumference
constexpr static int NUM_CHAMBERS = 1; //hero is weird, it indexes differently. Physically is 8.
constexpr static float GEAR_RATIO = 36.0f; //need to check if it is 36 exactly or 36ish
constexpr static float REV_PER_BALL = 42.0f / (50.0f * 3.1415926353f) * 2.0f; // GUESS, this value should be tuned since it isn't exactly known
constexpr static float REV_PER_BALL_BOTTOM = 1; // revolutions per ball = gearbox * external reduction / chambers
constexpr static float LOAD_BALL_PER_SECOND = 5.0f;
constexpr static float UNJAM_BALL_PER_SECOND = -8.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};
constexpr static float INITIAL_INDEX_OFFSET = 0; //doesn't apply to hero
constexpr static float INDEXING_EXTRA_BALLS = 0.2; //balls, how long to keep shooting after the beam break says the ball has left
#elif defined(SENTRY)
constexpr static int NUM_CHAMBERS = 7;
constexpr static float GEAR_RATIO = 36.0f; //need to check if it is 36 exactly or 36ish
constexpr static float REV_PER_BALL = GEAR_RATIO / NUM_CHAMBERS; // revolutions per ball = ratio / chambers
constexpr static float UNJAM_BALL_PER_SECOND = -1.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};
constexpr static float INITIAL_INDEX_OFFSET = 0.93f;
#elif defined(INFANTRY)
constexpr static int NUM_CHAMBERS = 8;
constexpr static float GEAR_RATIO = 36.0f; //need to check if it is 36 exactly or 36ish
constexpr static float REV_PER_BALL = GEAR_RATIO / NUM_CHAMBERS; // revolutions per ball = ratio / chambers
constexpr static float UNJAM_BALL_PER_SECOND = -1.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};
constexpr static float INITIAL_INDEX_OFFSET = 0.49f; //0.5 is barely too much
#else
constexpr static int NUM_CHAMBERS = 7;
constexpr static float GEAR_RATIO = 36.0f; //need to check if it is 36 exactly or 36ish
constexpr static float REV_PER_BALL = GEAR_RATIO / NUM_CHAMBERS; // revolutions per ball = ratio / chambers
constexpr static float UNJAM_BALL_PER_SECOND = -1.0f; // in unjam mode, spin backwards at 1 balls per second (this is a guess)
constexpr static tap::algorithms::SmoothPidConfig PID_CONF_INDEX = {5, 0, 0, 0, 8000, 1, 0, 1, 0, 10, 0};
#endif

#ifndef HERO
constexpr static float REV_PER_BALL_BOTTOM = 0; //doesn't apply to non heros
constexpr static float LOAD_BALL_PER_SECOND = 0; //doesn't apply to non heros
constexpr static float INDEXING_EXTRA_BALLS = 0;
#endif