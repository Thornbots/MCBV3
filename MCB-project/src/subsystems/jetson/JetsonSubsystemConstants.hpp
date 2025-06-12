#pragma once

#include "modm/math/geometry/vector.hpp"


#if defined(INFANTRY)
//distance to the camera in frame 4
//-.0175

static constexpr float cameraXoffset = -.0325; //appears to be RGB offset, not depth offset
static constexpr float cameraYoffset = .1279;
static constexpr float cameraZoffset = .04505;

static constexpr float initialShotVelocity = 24.0;           // Shot velocity


static constexpr float l = 0.05;           // Combined camera + Jetson latency
static constexpr float deltaTime = 0.033;  // Frame time
static constexpr float H = 9.95;   

#elif defined(SENTRY)

//distance to the camera in frame 4
//-.0175
static constexpr float cameraXoffset = -.0325; //appears to be RGB offset, not depth offset
static constexpr float cameraYoffset = .16034;
static constexpr float cameraZoffset = .1295;


static constexpr float initialShotVelocity = 22.0;           // Shot velocity

static constexpr float l = 0.05;           // Combined camera + Jetson latency
static constexpr float deltaTime = 0.033;  // Frame time
static constexpr float H = 9.95;   



#elif defined(HERO)

//distance to the camera in frame 4
//-.0175
static constexpr float cameraXoffset = -.0325; //appears to be RGB offset, not depth offset
static constexpr float cameraYoffset = .180857;
static constexpr float cameraZoffset = .048;


static constexpr float initialShotVelocity = 12.5;           // Shot velocity

static constexpr float l = 0.05;           // Combined camera + Jetson latency
static constexpr float deltaTime = 0.033;  // Frame time
static constexpr float H = 9.95;   



#else

static constexpr float cameraXoffset = -.0325; //appears to be RGB offset, not depth offset
static constexpr float cameraYoffset = .1279;
static constexpr float cameraZoffset = .04505;


static constexpr float initialShotVelocity = 24.0;           // Shot velocity

static constexpr float l = 0.05;           // Combined camera + Jetson latency
static constexpr float deltaTime = 0.033;  // Frame time
static constexpr float H = 9.95;   


#endif