#include "JoystickMoveCommand.hpp"

#include "GimbalSubsystemConstants.hpp"

namespace commands {

void JoystickMoveCommand::initialize() { firstTime = true; }
void JoystickMoveCommand::execute() {
    executeWith(drivers, gimbal, firstTime, isOffset);
}

void JoystickMoveCommand::executeWith(src::Drivers* drivers, GimbalSubsystem* gimbal) {
    bool firstTime = false;
    bool isOffset = false;
    executeWith(drivers, gimbal, firstTime, isOffset);
}
void JoystickMoveCommand::executeWith(src::Drivers* drivers, GimbalSubsystem* gimbal, bool& firstTime, bool& isOffset) {
    float yawInc = CONTROLLER_YAW_PROPORTIONAL * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
    float rawControllerPitch = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    float pitchInc = CONTROLLER_PITCH_PROPORTIONAL * rawControllerPitch;
    
    bool recalibrationInput = isOffset && rawControllerPitch < PITCH_RECAL_THRESHOLD;
    
    if (recalibrationInput) {
        if (isRecalibrating) {
            if (!hasRecalibrated && tap::arch::clock::getTimeMilliseconds() - recalStartedTime >= RECAL_HOLD_TIME) {
                gimbal->setPrevTargetPitch(SECOND_PITCH_OFFSET);
                drivers->recal.forceCalibration();
                hasRecalibrated = true;
            }
        } else {
            isRecalibrating = true;
            recalStartedTime = tap::arch::clock::getTimeMilliseconds();
        }
    } else {
        isRecalibrating = false;
        hasRecalibrated = false;
    }

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