#include "MouseMoveCommand.hpp"
#include "GimbalSubsystemConstants.hpp"

namespace commands {

void MouseMoveCommand::initialize() {
    mouseXOffset = drivers->remote.getMouseX();
    mouseYOffset = drivers->remote.getMouseY();
}
void MouseMoveCommand::execute() {
    yaw = MOUSE_YAW_PROPORTIONAL * (drivers->remote.getMouseX() - mouseXOffset);
    pitch += MOUSE_PITCH_PROPORTIONAL * (drivers->remote.getMouseY() - mouseYOffset);

    gimbal->updateMotors(yaw, &pitch);
}

void MouseMoveCommand::end(bool) { pitch = 0; }

bool MouseMoveCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands