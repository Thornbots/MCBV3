#include "AutoAimCommand.hpp"

namespace commands {
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
    }
}

void AutoAimCommand::end(bool) { pitch = 0; }

bool AutoAimCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands