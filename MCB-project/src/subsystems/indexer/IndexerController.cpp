#include "IndexerController.hpp"

#include <cmath>
#include <iostream>
#include <string>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

namespace subsystems {
using namespace subsystems::indexer;
    
IndexerController::IndexerController() {}

float IndexerController::calculate(float currentPosition, float currentVelocity, float targetPosition, float inputVelocity, float deltaT) {

    // float positionError = std::fmod(targetPosition - currentPosition + PI, 2 * PI) - PI;  // wrap to [-PI, PI]
    float positionError = targetPosition - currentPosition;  // wrap to [-PI, PI]

    /*
    while(positionError > PI) {
        positionError -= 2 * PI;
    }
    while (positionError < -PI) {
        positionError += 2 * PI;
    } */

    float targetVelocity = decelProfile(positionError, currentVelocity, inputVelocity);

    // experimental

    // model based motion profile
    float frictionTorque = (- C *  currentVelocity + UK * signum(-currentVelocity));
    float backEMF = -KB*RATIO * currentVelocity;
    float maxVelocity = std::min(VELO_MAX, 
        pastTargetVelocity + 1 / J * (frictionTorque 
            + std::min(std::max((backEMF + VOLT_MAX ) / RA, -CURRENT_MAX), CURRENT_MAX) //current from applied voltage, clamped to current max
         * KT * RATIO) * A_SCALE * deltaT);
    float minVelocity = std::max(-VELO_MAX, 
        pastTargetVelocity + 1 / J * (frictionTorque 
            + std::min(std::max((backEMF - VOLT_MAX ) / RA, -CURRENT_MAX), CURRENT_MAX)
         * KT * RATIO) * A_SCALE * deltaT);
    targetVelocity = std::clamp(targetVelocity, minVelocity, maxVelocity);

    // velocity controller
    float velocityError = targetVelocity - currentVelocity;
    float targetAcceleration = (targetVelocity - pastTargetVelocity) / deltaT;
    pastTargetVelocity = targetVelocity;

    // integral velocity controller
    if (std::abs(pastOutput) < INT_THRESH || velocityError * buildup < 0)  // signs are opposite
    {                                                                 // saturation detection
         if (velocityError * buildup < 0) {                              // overshooting
        buildup *= (1 - TAKEBACK);  // take back not quite half
         }
        buildup += velocityError * deltaT;  // integrate normally
    }   
    // calculation for setting target current aka velocity controller
    float targetCurrent = std::clamp(KVISC * targetVelocity + UK * signum(targetVelocity) + KA * targetAcceleration + KPV * velocityError + KIV * buildup, -20.0f, 20.0f);

    pastOutput = RA * targetCurrent + KV * targetVelocity;

    return targetCurrent;
}

float IndexerController::decelProfile(float poserror, float thetadot, float thetadotinput) {
    float o = 0, o2 = 0;  // offsets
    float t2 = thetadotinput * thetadotinput;
    float v1 = 0, v2 = 0, v3 = 0, v4 = 0;

    o = (thetadotinput - THETA_DOT_BREAK) / KP + (THETA_DOT_BREAK * THETA_DOT_BREAK - t2) / (2 * A_DECEL);
    o2 = (thetadotinput + THETA_DOT_BREAK) / KP + (t2 - THETA_DOT_BREAK * THETA_DOT_BREAK) / (2 * A_DECEL);

    if (t2 - 2 * A_DECEL * (-poserror - o) >= 0) {
        v1 = std::sqrt(t2 - 2 * A_DECEL * (-poserror - o));   // Positive left
        v2 = -std::sqrt(t2 - 2 * A_DECEL * (-poserror - o));  // Negative left
    }
    if (t2 - 2 * A_DECEL * (poserror + o2) >= 0) {
        v3 = std::sqrt(t2 - 2 * A_DECEL * (poserror + o2));   // Positive right
        v4 = -std::sqrt(t2 - 2 * A_DECEL * (poserror + o2));  // Negative right
    }
    if (std::fabs(poserror) < THETA_DOT_BREAK / KP)  // std::fabs(thetadot - thetadotinput) < THETA_DOT_BREAK)
        return KP * poserror + thetadotinput;

    if (v3 != 0 && poserror > 0 && thetadot <= 0)
        return v3;
    else if (v2 != 0 && poserror < 0 && thetadot >= 0)
        return v2;
    else if (poserror > 0)
        return v1;
    else
        return v4;
}
}  // namespace subsystems