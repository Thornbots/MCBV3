#include "AutoAimCommand.hpp"

namespace commands {
int shoot = 0;
void AutoAimCommand::initialize() {
    shoot = -1;

}
void AutoAimCommand::execute() {
    float dyaw = 0;
    cv->update(yaw, pitch, &dyaw, &pitch, &shoot);
    if(shoot!=-1)
        gimbal->updateMotors(-dyaw/2, &pitch);
    else 
        gimbal->updateMotors(0, &pitch);

    if(shoot==1){
        indexer->indexAtRate(10);
    } else {
        indexer->stopIndex();
    }
}

void AutoAimCommand::end(bool) { pitch = 0; }

bool AutoAimCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands