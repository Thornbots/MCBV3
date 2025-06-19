#include "MouseMoveCommand.hpp"
#include "GimbalSubsystemConstants.hpp"

namespace commands {

void MouseMoveCommand::initialize() {
}

void MouseMoveCommand::execute() {
    float yawInc = MOUSE_YAW_PROPORTIONAL * (drivers->remote.getMouseX());
    float pitchInc = MOUSE_PITCH_PROPORTIONAL * (drivers->remote.getMouseY());

    gimbal->updateMotors(yawInc, gimbal->getPrevTargetPitch() + pitchInc);
}

void MouseMoveCommand::end(bool) {}

bool MouseMoveCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands