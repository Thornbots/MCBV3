#pragma once

#include "modm/math/geometry/vector.hpp"

// here tune confidence
static constexpr float MSG_CONFIDENCE_CUTOFF = 0.75f; //lower than this confidence is ignored

//here tune allow 'cone' (we are just checking yaw)
static constexpr float YAW_OUT_SHOOT_THRESH = PI / 8; //if cv wants to move the yaw less than this amount, allow shooting

static constexpr float YAW_CLOSE = 0.01f;


#if defined(INFANTRY)
//distance to the camera in frame 4
//-.0175

// find new constants for std and hero
static constexpr float cameraXoffset = -.0325; // side to side; appears to be RGB offset, not depth offset
static constexpr float cameraYoffset = 131.826/1000; // front back (front of the camera (not include camera cover) to center of pitch axis)
static constexpr float cameraZoffset = 44.55/1000; //pretty sure z is up down (stedman)

static constexpr float initialShotVelocity = 24.0;           // Shot velocity

// static constexpr float l = 0.02;           // Combined camera + Jetson latency
// static constexpr float deltaTime = 0.0166;  // Frame time
static constexpr float H = 9.95;   

//here tune pitch. dyaw gets scaled by an amount proportional to dyaw
static constexpr float PITCH_MULTIPLY_MIN = 0.0f;
static constexpr float PITCH_MULTIPLY_MAX = 0.25f; //smaller would prevent overshoot
static constexpr float PITCH_MULTIPLY_SCALE = 0.5f; //mult agains dyaw (in rad, max is pi)

//here tune yaw. dyaw gets scaled by an amount proportional to dyaw
static constexpr float YAW_MULTIPLY_MIN = 0.0f;
static constexpr float YAW_MULTIPLY_MAX = 0.25f;
static constexpr float YAW_MULTIPLY_SCALE = 0.5f; //mult agains dyaw (in rad, max is pi)

#elif defined(SENTRY)

//distance to the camera in frame 4
//-.0175
static constexpr float cameraXoffset = -.0325; //appears to be RGB offset, not depth offset
static constexpr float cameraYoffset = 0.1;
static constexpr float cameraZoffset = 54.791/1000; //pretty sure z is up down (stedman)


static constexpr float initialShotVelocity = 24.0;           // Shot velocity

static constexpr float l = 0.02;           // Combined camera + Jetson latency
static constexpr float deltaTime = 0.0166;  // Frame time
static constexpr float H = 9.95;   

//here tune pitch
static constexpr float PITCH_DIVIDE = 3.0f; //higher number is less responsive

//here tune yaw
static constexpr float YAW_DIVIDE_FAR = 1.0f; //higher number is less responsive
static constexpr float YAW_DIVIDE_CLOSE = 1.0f; //higher number is less responsive


#elif defined(HERO)

//distance to the camera in frame 4
//-.0175
static constexpr float cameraXoffset = -.0325; //appears to be RGB offset, not depth offset
static constexpr float cameraYoffset = 209.5/1000;
static constexpr float cameraZoffset = 48.3/1000; //pretty sure z is up down (stedman)


static constexpr float initialShotVelocity = 14.0f;           // Shot velocity

static constexpr float l = 0.05;           // Combined camera + Jetson latency
static constexpr float deltaTime = 0.033;  // Frame time
static constexpr float H = 9.95;   

//here tune pitch
static constexpr float PITCH_DIVIDE = 3.0f; //higher number is less responsive

//here tune yaw
static constexpr float YAW_DIVIDE_FAR = 1.0f; //higher number is less responsive
static constexpr float YAW_DIVIDE_CLOSE = 1.0f; //higher number is less responsive


#else

static constexpr float cameraXoffset = -.0325; //appears to be RGB offset, not depth offset
static constexpr float cameraYoffset = .1279;
static constexpr float cameraZoffset = .04505; //pretty sure z is up down (stedman)


static constexpr float initialShotVelocity = 24.0;           // Shot velocity

static constexpr float l = 0.05;           // Combined camera + Jetson latency
static constexpr float deltaTime = 0.033;  // Frame time
static constexpr float H = 9.95;   

//here tune pitch
static constexpr float PITCH_DIVIDE = 3.0f; //higher number is less responsive

//here tune yaw
static constexpr float YAW_DIVIDE_FAR = 2.0f; //higher number is less responsive
static constexpr float YAW_DIVIDE_CLOSE = 2.0f; //higher number is less responsive


#endif