
#pragma once
#include "modm/math/geometry/vector.hpp"


#if defined(INFANTRY)
//distance to the camera in frame 4
//-.0175

float cameraXoffset = -.0325; //appears to be RGB offset, not depth offset
float cameraYoffset = .1279;
float cameraZoffset = .04505;


const float J = 24.0;           // Shot velocity

const float l = 0.05;           // Combined camera + Jetson latency
const float deltaTime = 0.033;  // Frame time
const float H = 9.95;   

#elif defined(SENTRY)

//distance to the camera in frame 4
//-.0175
float cameraXoffset = -.0325; //appears to be RGB offset, not depth offset
float cameraYoffset = .16034;
float cameraZoffset = .1295;


const float J = 22.0;           // Shot velocity

const float l = 0.05;           // Combined camera + Jetson latency
const float deltaTime = 0.033;  // Frame time
const float H = 9.95;   



#elif defined(HERO)

//distance to the camera in frame 4
//-.0175
float cameraXoffset = -.0325; //appears to be RGB offset, not depth offset
float cameraYoffset = .180857;
float cameraZoffset = .048;


const float J = 12.5;           // Shot velocity

const float l = 0.05;           // Combined camera + Jetson latency
const float deltaTime = 0.033;  // Frame time
const float H = 9.95;   



#else

float cameraXoffset = -.0325; //appears to be RGB offset, not depth offset
float cameraYoffset = .1279;
float cameraZoffset = .04505;


const float J = 24.0;           // Shot velocity

const float l = 0.05;           // Combined camera + Jetson latency
const float deltaTime = 0.033;  // Frame time
const float H = 9.95;   


#endif