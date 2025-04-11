#include "AutoAimAndFireCommand.hpp"

namespace commands {
void AutoAimAndFireCommand::initialize() { shoot = -1; }
void AutoAimAndFireCommand::execute() {
    float dyaw = 0;
    cv->update(yaw, pitch, &dyaw, &pitch, &shoot);
    // pitch *= 2;
    // moving gimbal

    if (shoot != -1) {
        //moving to panel
        gimbal->updateMotors(-dyaw / 4, pitch);
        lastSeenTime = tap::arch::clock::getTimeMilliseconds();
        if(shoot==1) isShooting = true;
    } else if (tap::arch::clock::getTimeMilliseconds() - lastSeenTime<2000) {
        //waiting for a bit, don't change isShooting
        gimbal->updateMotors(0, pitch);
    } else {
        //patrol, don't shoot
        isShooting = false;
        gimbal->updateMotors(0.002, pitch);
    }

    if (isShooting) {
        //if we see a panel or recently have seen a panel
        indexer->indexAtRate(10);
    } else {
        //if we haven't seen a panel for a bit
        indexer->stopIndex();
    }
}

void AutoAimAndFireCommand::end(bool) { pitch = 0; }

bool AutoAimAndFireCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands