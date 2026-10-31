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

// Field frame (REP-105, z up): 2026 spooky scrim 2v2 field, 8.384 m x 5.0 m, (0, 0) at
// the centre, x toward blue's base, y left. Both chassis boot facing +y; routes use the
// inward direction from each base. Start is red's (-3.53, -0.426), mirrored in x for
// blue, read from the SPOOKY_SCRIM_2V2 route drawing; not measured on the field yet.
// TEMPORARY, not good: scrim-only values copied to the Jetson and sim. Measure, or
// restore the ARCC field before the next event (docs/ROADMAP.md short todos).
static constexpr float FIELD_LENGTH = 8.384f;  // meters, along x
static constexpr float FIELD_WIDTH = 5.0f;     // meters, along y
static constexpr float START_X = 3.53f;        // meters
static constexpr float START_Y = -0.426f;      // meters
static constexpr float BOOT_YAW = 1.5707963267948966f;  // radians, both teams

}
