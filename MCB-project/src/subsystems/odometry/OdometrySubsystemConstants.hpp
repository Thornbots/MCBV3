#pragma once
/*
Constants file for all robots subsystem constants
*/
namespace odo{
constexpr static float PI_CONST = 3.14159;
constexpr static int ODO_MOTOR_MAX_VOLTAGE = 24000;  // Should be the voltage of the battery. Unless the motor maxes out below that.
static constexpr float dt = 0.002f;
// for sysid


constexpr static int ODO_MOTOR_MAX_SPEED = 1000;  // TODO: Make this value relevent
                                                  // //TODO: Check the datasheets

static constexpr float ODO_OFFSET = 180.0f/180.0f*PI_CONST;//90*PI_CONST/180;

static constexpr int ODO_DIST_RANGE = 18000;

}