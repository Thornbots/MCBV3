#include "AutoAimAndFireCommand.hpp"

namespace commands {
using namespace tap::communication::serial;

void AutoAimAndFireCommand::initialize() {
    shoot = -1;

    drivers->leds.set(tap::gpio::Leds::Green, true);
}
void AutoAimAndFireCommand::execute() {
    bool allowShooting = true;
    bool allowTracking = true;
    bool allowPatrol = true;

    bool allowGimbal = false;  // set if see a target and allow tracking, or if allowed patrolling

    if (drivers->refWrapper.isIn3v3()) {
        allowShooting = false;
        allowTracking = false;
        allowPatrol = false;

        if (drivers->refWrapper.isInGame()) {
            // allow all
            allowShooting = true;
            allowTracking = true;
            allowPatrol = true;
        }

        if (drivers->refWrapper.isBefore15Sec()) {
            // before match, only allow tracking for testers
            allowTracking = true;
        }

        if (drivers->refWrapper.isIn15Sec() || drivers->refWrapper.isBefore15Sec()) {
            // in 15 sec countdown, allow patrol but not shooting
            allowTracking = true;
            allowPatrol = true;
        }
    }

    float dyaw = 0;
    float currentYaw = gimbal->getYawAngleRelativeWorld();
    float currentPitch = gimbal->getPitchEncoderValue();
    cv->update(currentYaw, currentPitch, yawvel, pitchvel, &dyaw, &pitch, &yawvel, &pitchvel, &shoot);

    if (shoot != -1) {
        if (tap::arch::clock::getTimeMilliseconds() - lastSeenTime > PERSISTANCE) flip = flip * -1;
        // Found a target, moving to it and maybe shooting at it

        dyaw = fmod(dyaw, 2 * PI);
        // clamp between -Pi and PI to allow for dividing
        dyaw = dyaw > PI ? dyaw - 2 * PI : dyaw < -PI ? dyaw + 2 * PI : dyaw;
        lastSeenTime = tap::arch::clock::getTimeMilliseconds();

        dyaw = std::clamp(dyaw, -.2f, .2f);
        if (allowTracking) {
            gimbal->updateMotorsAndVelocityWithLatencyCompensation(dyaw / 2.5f, pitch, yawvel, pitchvel);  // division is to prevent overshoot from latency
            allowGimbal = true;
        }
        if (shoot == 1) isShooting = true;
    } else if (tap::arch::clock::getTimeMilliseconds() - lastSeenTime < PERSISTANCE) {
        // Haven't found a target right now but I have recently, keep shooting if I was shooting

        if (allowTracking) {
            gimbal->updateMotors(0, pitch);
            allowGimbal = true;
        }
    } else {
        // Haven't found a target, patrol

        isShooting = false;
        pitch = 0.05;  // pitch down to avoid looking into the sky
        numCyclesForBurst++;

        if (allowPatrol) {
            if (numCyclesForBurst == CYCLES_UNTIL_BURST) {
                gimbal->updateMotors(flip * BURST_AMOUNT, pitch);
                numCyclesForBurst = 0;
            } else {
                gimbal->updateMotors(flip * PATROL_SPEED, pitch);
            }
        }
    }

    if (allowShooting) {
        if (isShooting) {
            // if we see a panel or recently have seen a panel
            indexer->indexAtRate(shotsPerSecond);
        } else {
            // if we haven't seen a panel for a bit
             indexer->stopIndex();
            // indexer->unjam();
        }
    } else {
        indexer->stopIndex();
    }

    if (allowPatrol) {
        flywheel->setTargetVelocity(FLYWHEEL_MOTOR_MAX_RPM);
    } else {
        if (adc->getIsScheduled()) flywheel->setTargetVelocity(FLYWHEEL_MOTOR_MAX_RPM / 4); //hum to let setupers know setup right
    }

    if(!allowGimbal)
        gimbal->stopMotors();
}

void AutoAimAndFireCommand::end(bool) {
    pitch = 0;
    drivers->leds.set(tap::gpio::Leds::Green, false);
}

bool AutoAimAndFireCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands