#pragma once
/*
Constants file for all robots subsystem constants
*/
namespace odo{
constexpr static float PI_CONST = 3.14159;
constexpr static int ODO_MOTOR_MAX_VOLTAGE = 24000;  // Should be the voltage of the battery. Unless the motor maxes out below that.
static constexpr float dt = 0.001f;
// for sysid


constexpr static int ODO_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                  // //TODO: Check the datasheets

static constexpr float ODO_OFFSET = -0.25 + PI_CONST/2;//90*PI_CONST/180;

static constexpr int ODO_DIST_RANGE = 18000;

// Field frame (REP-105, z up): 12 m x 8 m, (0, 0) at the centre, x toward blue's base,
// y left. Red boots at (-START_X, START_Y) facing +x, blue at (START_X, START_Y) facing -x.
// START_X puts ARCC_ROUGH_PATH's centre target on x = 0; not measured on the field yet.
static constexpr float FIELD_LENGTH = 12.0f;  // meters, along x
static constexpr float FIELD_WIDTH = 8.0f;    // meters, along y
static constexpr float START_X = 4.625f;      // meters
static constexpr float START_Y = 0.0f;        // meters

}