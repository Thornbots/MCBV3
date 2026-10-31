#pragma once
/*
Constants file for all robots subsystem constants
*/
constexpr static float PI_CONST = 3.14159;
constexpr static int YAW_MOTOR_MAX_VOLTAGE = 24000;  // Should be the voltage of the battery. Unless the motor maxes out below that.
// main.cpp and the hosted scheduler refresh controls every 1 ms.
static constexpr float dt = 0.001f;
// for sysid
static constexpr int PITCH_DIST_RANGE = 0;

static constexpr float MOUSE_YAW_PROPORTIONAL = -0.00008;
static constexpr float MOUSE_PITCH_PROPORTIONAL = 0.00005;

static constexpr float CONTROLLER_YAW_PROPORTIONAL = -0.005;
static constexpr float CONTROLLER_PITCH_PROPORTIONAL = -0.005;

static constexpr int LATENCY_Q_SIZE = 23; //12 cycles = 24ms

static constexpr float PITCH_RECAL_THRESHOLD = -0.5;
static constexpr float PITCH_MAPPING_THRESHOLD = 0.5;
static constexpr float RECAL_HOLD_TIME = 3000; // milliseconds to hold controller combo until recalibration
static constexpr float MAPPING_HOLD_TIME = 5000; // milliseconds to hold controller combo until sentry enters mapping mode

#if defined(HERO)
constexpr static int YAW_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                  // //TODO: Check the datasheets

static constexpr float MAX_PITCH_UP = PI_CONST / 180 * 55;
static constexpr float MAX_PITCH_DOWN = PI_CONST / 180 * 30;

static constexpr float YAW_OFFSET =  0.75 * PI_CONST ;

static constexpr float PITCH_OFFSET = (0.01 ) * PI_CONST;  // to make gimbal horizontal when told to go to 0

static constexpr float SECOND_PITCH_OFFSET = 0.85f;  // to make gimbal look down to fit sizing box

static constexpr float YAW_TOTAL_RATIO = 32319.0f / 748.0f;  // unitless, ratio of encoder counts to degrees of rotation

static constexpr int YAW_DIST_RANGE = 18000;

static constexpr float PITCH_RATIO = 4.0f;

#elif defined(SENTRY)
constexpr static int YAW_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                  // //TODO: Check the datasheets

static constexpr float MAX_PITCH_UP = PI_CONST / 180 * 20;
static constexpr float MAX_PITCH_DOWN = PI_CONST / 180 * 21;

static constexpr float YAW_OFFSET = 7*PI/4;


//-1.62f = 2.15
//0.0f   = 0.52
static constexpr float PITCH_OFFSET = 0.52f;  // to make gimbal horizontal when told to go to 0

static constexpr float SECOND_PITCH_OFFSET = 0.1f;

static constexpr float YAW_TOTAL_RATIO = 32319.0f / 748.0f;  // unitless, ratio of encoder counts to degrees of rotation

static constexpr int YAW_DIST_RANGE = 18000;

static constexpr float PITCH_RATIO = 3.0f;

#elif defined(INFANTRY)
constexpr static int YAW_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                  // //TODO: Check the datasheets

static constexpr float MAX_PITCH_UP = PI_CONST / 180 * 20;
static constexpr float MAX_PITCH_DOWN = PI_CONST / 180 * 21;

static constexpr float YAW_OFFSET = 0.0f;

static constexpr float PITCH_OFFSET = 1.26;  // to make gimbal horizontal when told to go to 0

static constexpr float SECOND_PITCH_OFFSET = 0.08 * PI_CONST;

static constexpr float YAW_TOTAL_RATIO = 32319.0f / 748.0f;  // unitless, ratio of encoder counts to degrees of rotation

static constexpr int YAW_DIST_RANGE = 16384 / 4;  // 20/4 = 5 amps

static constexpr float PITCH_RATIO = 3.0f;

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

static constexpr float SECOND_PITCH_OFFSET = 0.08 * PI_CONST;

static constexpr int YAW_DIST_RANGE = 18000;

static constexpr float PITCH_RATIO = 1.0f;

#endif