#include "JoystickMoveCommand.hpp"
#include "GimbalSubsystemConstants.hpp"

namespace commands
{

void JoystickMoveCommand::initialize() {  

}
void JoystickMoveCommand::execute()
{
        yaw = CONTROLLER_YAW_PROPORTIONAL * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
        pitch = CONTROLLER_PITCH_PROPORTIONAL * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL); //in the future, use the ranges from GimbalSubsystemConstants
  
        //TODO this lmao
        gimbal->updateMotors(yaw, &pitch);

}

void JoystickMoveCommand::end(bool) {}

bool JoystickMoveCommand::isFinished(void) const { return !drivers->remote.isConnected(); }
}  // namespace commands