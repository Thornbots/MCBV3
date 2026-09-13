#include "JoystickMoveCommand.hpp"

#include "GimbalSubsystemConstants.hpp"

namespace commands {

void JoystickMoveCommand::initialize() { firstTime = true; }
void JoystickMoveCommand::execute() {
    float yawInc = CONTROLLER_YAW_PROPORTIONAL * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
    float pitchInc = CONTROLLER_PITCH_PROPORTIONAL * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);

    if (firstTime) {
        firstTime=false;
        if (isOffset) {
            pitchInc += SECOND_PITCH_OFFSET;
        }
        gimbal->updateMotors(yawInc, pitchInc);
    } else {
        gimbal->updateMotors(yawInc, gimbal->getPrevTargetPitch() + pitchInc);
    }
}

void JoystickMoveCommand::end(bool) {}

bool JoystickMoveCommand::isFinished(void) const { return !drivers->remote.isConnected(); }
}  // namespace commands