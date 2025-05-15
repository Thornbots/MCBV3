#include "AutoAimAndFireCommand.hpp"

namespace commands {
void AutoAimAndFireCommand::initialize() { shoot = -1; }
void AutoAimAndFireCommand::execute() {
    float dyaw = 0;
    float currentYaw = gimbal->getYawAngleRelativeWorld();
    float currentPitch = gimbal->getPitchEncoderValue();
    cv->update(currentYaw, currentPitch, yawvel, pitchvel, &dyaw, &pitch, &yawvel, &pitchvel, &shoot);

    if (shoot != -1) {
        dyaw = fmod(dyaw, 2*PI);
        //clamp between -Pi and PI to allow for dividing
        dyaw = dyaw > PI ? dyaw - 2*PI : dyaw < -PI ? dyaw + 2*PI : dyaw; 
        gimbal->updateMotorsAndVelocity(dyaw / 4.0f, pitch, yawvel, pitchvel); //division is to prevent overshoot from latency
        lastSeenTime = tap::arch::clock::getTimeMilliseconds();
        if(shoot==1) isShooting = true;
    } else if (tap::arch::clock::getTimeMilliseconds() - lastSeenTime<1500) {
        //waiting for a bit, don't change isShooting
        gimbal->updateMotors(0, pitch);
    } else {
        //patrol, don't shoot
        isShooting = false;
        pitch = 0.05; //pitch down to avoid looking into the sky
        numCyclesForBurst++;
        if(numCyclesForBurst==CYCLES_UNTIL_BURST){
            gimbal->updateMotors(0.5, pitch);
            numCyclesForBurst = 0;
        } else{
            gimbal->updateMotors(0.003, pitch);
        }
    }

    if (isShooting) {
        //if we see a panel or recently have seen a panel
        indexer->indexAtRate(20);
    } else {
        //if we haven't seen a panel for a bit
        // indexer->stopIndex();
        indexer->unjam();
    }
}

void AutoAimAndFireCommand::end(bool) { pitch = 0; }

bool AutoAimAndFireCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands