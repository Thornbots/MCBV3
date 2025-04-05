#include "PitchController.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include <bits/stdc++.h>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "PitchControllerConstants.hpp"


namespace subsystems
{
PitchController::PitchController() {}

float PitchController::calculate(float currentPos, float currentVelo, float targetPos, float deltaT)
{
    float positionError = targetPos - currentPos;
    
    float targetVelo = KP * positionError + (targetPos - pastTarget) / deltaT;

    // model based motion profile
    float maxVelocity = std::min(VELO_MAX, pastTargetVelo + ACCEL_MAX * deltaT);
    float minVelocity = std::max(-VELO_MAX, pastTargetVelo - ACCEL_MAX * deltaT);
    targetVelo = std::clamp(targetVelo, minVelocity, maxVelocity);

    // velocity controller
    float velocityError = targetVelo - currentVelo;
    pastTargetVelo = targetVelo;

    // integral velocity controller
    if (abs(pastOutput) < INT_THRESH || velocityError * buildup < 0)
    {  // saturation detection
        if (velocityError * buildup < 0)
        {                               // overshooting
            buildup *= (1 - TAKEBACK);  // take back not quite half
        }
        else
        {
            buildup += velocityError * deltaT;  // integrate normally
        }
    }
    // calculation for setting target current aka velocity controller
    float targetCurrent = KSTATIC * signum(targetVelo) + KF + KPV * velocityError + KIV * buildup;

    pastOutput = RA * targetCurrent + KV * targetVelo;
    pastTarget = targetPos;
    return std::clamp(pastOutput, -VOLT_MAX, VOLT_MAX);
}
}  // namespace subsystems
