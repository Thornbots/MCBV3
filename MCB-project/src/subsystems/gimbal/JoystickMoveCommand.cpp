#include "JoystickMoveCommand.hpp"

#include "GimbalSubsystemConstants.hpp"

namespace commands {

void JoystickMoveCommand::initialize() {}
void JoystickMoveCommand::execute() {
    yaw = CONTROLLER_YAW_PROPORTIONAL * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL);
    float rawControllerPitch = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    pitch = CONTROLLER_PITCH_PROPORTIONAL * rawControllerPitch;  // in the future, use the ranges from GimbalSubsystemConstants

    if (!(isOffset && rawControllerPitch < PITCH_RECAL_THRESHOLD)) {
        isRecalibrating = false;
        hasRecalibrated = false;
    }

    if (isOffset) {
        if (rawControllerPitch < PITCH_RECAL_THRESHOLD) {
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
        }

        pitch += SECOND_PITCH_OFFSET;
    }

    gimbal->updateMotors(yaw, pitch);
    // TODO this lmao
}

void JoystickMoveCommand::end(bool) {}

bool JoystickMoveCommand::isFinished(void) const { return !drivers->remote.isConnected(); }
}  // namespace commands