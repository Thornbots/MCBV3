#pragma once

#include "modm/math/geometry/vector.hpp"

// here tune confidence
static constexpr float MSG_CONFIDENCE_CUTOFF = 0.75f; //lower than this confidence is ignored

//here tune allow 'cone' (we are just checking yaw)
static constexpr float YAW_OUT_SHOOT_THRESH = PI / 4; //if cv wants to move the yaw less than this amount, allow shooting

static constexpr float YAW_CLOSE = 0.05f;


#if defined(INFANTRY)
//distance to the camera in frame 4
//-.0175

static constexpr float cameraXoffset = -.0325; //appears to be RGB offset, not depth offset
static constexpr float cameraYoffset = .1279;
static constexpr float cameraZoffset = .04505;

static constexpr float initialShotVelocity = 24.0;           // Shot velocity

// static constexpr float l = 0.02;           // Combined camera + Jetson latency
// static constexpr float deltaTime = 0.0166;  // Frame time
static constexpr float H = 9.95;   

//here tune pitch
static constexpr float PITCH_DIVIDE = 1.5f; //higher number is less responsive

//here tune yaw
static constexpr float YAW_DIVIDE_FAR = 1.0f; //higher number is less responsive
static constexpr float YAW_DIVIDE_CLOSE = 1.0f; //higher number is less responsive

#elif defined(SENTRY)

//distance to the camera in frame 4
//-.0175
static constexpr float cameraXoffset = -.0325; //appears to be RGB offset, not depth offset
static constexpr float cameraYoffset = .16034;
static constexpr float cameraZoffset = .1295;


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
static constexpr float cameraYoffset = .180857;
static constexpr float cameraZoffset = .048;


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
static constexpr float cameraZoffset = .04505;


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