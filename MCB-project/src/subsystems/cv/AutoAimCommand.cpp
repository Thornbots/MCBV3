#include "AutoAimCommand.hpp"

namespace commands {
<<<<<<< HEAD
int shoot = 0;
void AutoAimCommand::initialize() { shoot = -1; }
void AutoAimCommand::execute() {
    float dyaw = 0;
    cv->update(yaw, pitch, &dyaw, &pitch, &shoot);
    
    // moving gimbal
    if (shoot != -1) {
        //moving to panel
        gimbal->updateMotors(-dyaw / 2, &pitch);
        lastSeenTime = tap::arch::clock::getTimeMicroseconds();
        if(shoot==1) isShooting = true;
    } else if (lastSeenTime<1000) {
        //waiting for a bit, don't change isShooting
        gimbal->updateMotors(0, &pitch);
    } else {
        //patrol, don't shoot
        isShooting = false;
        // drivers->leds.set()
        gimbal->updateMotors(0.002, &pitch);
    }

    if (isShooting) {
        //if we see a panel or recently have seen a panel
        indexer->indexAtRate(20);
    } else {
        //if we haven't seen a panel for a bit
        indexer->stopIndex();
=======
void AutoAimCommand::initialize() {
    shoot = -1;
    MouseMoveCommand::initialize();
}
void AutoAimCommand::execute() {
    float dyaw = 0;
    float currentYaw = gimbal->getYawAngleRelativeWorld();
    float currentPitch = gimbal->getPitchEncoderValue();
    cv->update(currentYaw, currentPitch, yawvel, pitchvel, &dyaw, &pitch, &yawvel, &pitchvel, &shoot);
    // moving gimbal
    if (shoot != -1) {
        // moving to panel

        dyaw = fmod(dyaw, 2*PI);
        //clamp between -Pi and PI to allow for dividing
        dyaw = dyaw > PI ? dyaw - 2*PI : dyaw < -PI ? dyaw + 2*PI : dyaw; 
        gimbal->updateMotorsAndVelocity(dyaw / 3.0f, pitch, yawvel, pitchvel); //division is to prevent overshoot from latency
        //gimbal->updateMotors(dyaw/3.0f, pitch);
    } else {
        // sentry's equivalent of patrol, do original mouse moving
        MouseMoveCommand::execute();
>>>>>>> ccfe370f24c6be7b6f2b61af37d748a78f2ab3d3
    }
}

void AutoAimCommand::end(bool interrupted) { MouseMoveCommand::end(interrupted); }

bool AutoAimCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands