#include "MouseMoveCommand.hpp"
#include "GimbalSubsystemConstants.hpp"

namespace commands {

void MouseMoveCommand::initialize() {
    pitch = gimbal->getPitchEncoderValue();
}
void MouseMoveCommand::execute() {
    yaw = MOUSE_YAW_PROPORTIONAL * (drivers->remote.getMouseX());
    pitch += MOUSE_PITCH_PROPORTIONAL * (drivers->remote.getMouseY());

    yawvel = gimbal->getYawVel();
    pitchvel = gimbal->getPitchVel(); 

    pitch = std::clamp(pitch, -MAX_PITCH_DOWN, MAX_PITCH_UP);
    gimbal->updateMotors(yaw, pitch);
}

void MouseMoveCommand::end(bool) { pitch = 0; }

bool MouseMoveCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands