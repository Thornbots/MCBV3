#include "AutoAimCommand.hpp"

namespace commands {
void AutoAimCommand::initialize() {
    shoot = -1;
    MouseMoveCommand::initialize();
}
void AutoAimCommand::execute() {
    float dyaw = 0;
    cv->update(yaw, pitch, &dyaw, &pitch, &shoot);
    // moving gimbal
    if (shoot != -1) {
        // moving to panel
        gimbal->updateMotors(-dyaw / 4, &pitch);
    } else {
        // sentry's equivalent of patrol, do original mouse moving
        MouseMoveCommand::execute();
    }
}

void AutoAimCommand::end(bool interrupted) { MouseMoveCommand::end(interrupted); }

bool AutoAimCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands