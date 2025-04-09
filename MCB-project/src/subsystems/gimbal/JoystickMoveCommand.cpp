#include "JoystickMoveCommand.hpp"

#include "GimbalSubsystemConstants.hpp"

namespace commands {

void JoystickMoveCommand::initialize() {}
void JoystickMoveCommand::execute() {
    yaw = CONTROLLER_YAW_PROPORTIONAL * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
    pitch = CONTROLLER_PITCH_PROPORTIONAL * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);  // in the future, use the ranges from GimbalSubsystemConstants

#if defined(INFANTRY)
    pitch -= drivers->bmi088.getRoll() * PI / 180;
#elif defined(HERO)
    // todo for hero lmao
#endif
    gimbal->updateMotors(yaw, &pitch);
    // TODO this lmao
}

void JoystickMoveCommand::end(bool) {}

bool JoystickMoveCommand::isFinished(void) const { return !drivers->remote.isConnected(); }
}  // namespace commands