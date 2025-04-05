#pragma once
/*
Constants file for all robots subsystem constants
*/
constexpr static float PI_CONST = 3.14159;
constexpr static int YAW_MOTOR_MAX_VOLTAGE = 24000;  // Should be the voltage of the battery. Unless the motor maxes out below that.
static constexpr float dt = 0.002f;
// for sysid
static constexpr int PITCH_DIST_RANGE = 0;

static constexpr float MOUSE_YAW_PROPORTIONAL = -0.00008;
static constexpr float MOUSE_PITCH_PROPORTIONAL = 0.00005;

static constexpr float CONTROLLER_YAW_PROPORTIONAL = -0.02;

#if defined(HERO)
constexpr static int YAW_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                  // //TODO: Check the datasheets

// standard looks down 17 degrees, 15 is safe
static constexpr float MAX_PITCH_UP = PI_CONST / 180 * 15;
// looks up 20, 18 is safe
static constexpr float MAX_PITCH_DOWN = PI_CONST / 180 * 18;

static constexpr float YAW_OFFSET = 3 * PI_CONST / 4;

static constexpr float PITCH_OFFSET = -0.48 * PI_CONST;  // to make gimbal horizontal when told to go to 0

static constexpr float YAW_TOTAL_RATIO = 32319.0f / 748.0f;  // unitless, ratio of encoder counts to degrees of rotation

static constexpr int YAW_DIST_RANGE = 18000;

static constexpr float CONTROLLER_PITCH_PROPORTIONAL = 0.1 * PI;

#elif defined(SENTRY)
constexpr static int YAW_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                  // //TODO: Check the datasheets

static constexpr float MAX_PITCH_UP = PI_CONST / 180 * 12*2; //*2 for ratio
static constexpr float MAX_PITCH_DOWN = PI_CONST / 180 * 12*2;

static constexpr float YAW_OFFSET = PI_CONST;

static constexpr float PITCH_OFFSET = 1.75f * PI_CONST;  // to make gimbal horizontal when told to go to 0

static constexpr float YAW_TOTAL_RATIO = 54.0f/27.0f * 3591.0f / 187.0f;  // unitless, ratio of encoder counts to degrees of rotation

static constexpr int YAW_DIST_RANGE = 18000;

static constexpr float CONTROLLER_PITCH_PROPORTIONAL = 0.2f * PI;

#elif defined(INFANTRY)
constexpr static int YAW_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                  // //TODO: Check the datasheets

// standard looks down 17 degrees, 15 is safe
static constexpr float MAX_PITCH_UP = PI_CONST / 180 * 15;
// looks up 20, 18 is safe
static constexpr float MAX_PITCH_DOWN = PI_CONST / 180 * 14;

static constexpr float YAW_OFFSET = 0;

static constexpr float PITCH_OFFSET = 1 * PI_CONST;  // to make gimbal horizontal when told to go to 0

static constexpr float YAW_TOTAL_RATIO = 32319.0f / 748.0f;  // unitless, ratio of encoder counts to degrees of rotation

static constexpr int YAW_DIST_RANGE = 16384 / 4;  // 20/4 = 5 amps

static constexpr float CONTROLLER_PITCH_PROPORTIONAL = 0.1 * PI;

#else
constexpr static int YAW_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                  // //TODO: Check the datasheets

// standard looks down 17 degrees, 15 is safe
static constexpr float MAX_PITCH_UP = PI_CONST / 180 * 15;
// looks up 20, 18 is safe
static constexpr float MAX_PITCH_DOWN = PI_CONST / 180 * 18;

static constexpr float YAW_OFFSET = 3 * PI_CONST / 4;

static constexpr float YAW_TOTAL_RATIO = 1.0f;  // unitless, ratio of encoder counts to degrees of rotation

static constexpr float PITCH_OFFSET = -0.48 * PI_CONST;  // to make gimbal horizontal when told to go to 0

static constexpr int YAW_DIST_RANGE = 18000;

static constexpr float CONTROLLER_PITCH_PROPORTIONAL = 0.1 * PI;

#endif