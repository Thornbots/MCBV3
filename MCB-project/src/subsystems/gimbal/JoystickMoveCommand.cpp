#include "JoystickMoveCommand.hpp"

#include "GimbalSubsystemConstants.hpp"

namespace commands {

void JoystickMoveCommand::initialize() { firstTime = true; }

void JoystickMoveCommand::execute() {
    // do joystick movement
    executeWith(drivers, gimbal, firstTime, isOffset);
    
    // do special behavior in the isOffset mode
    float rawControllerPitch = drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL);
    bool recalibrationInput = isOffset && rawControllerPitch < PITCH_RECAL_THRESHOLD; //point down to recalibrate
    bool mappingInput = isOffset && rawControllerPitch > PITCH_MAPPING_THRESHOLD; //point up to start mapping
    
    if (recalibrationInput) {
        if(downForRecalTimeout.isStopped()) downForRecalTimeout.restart(RECAL_HOLD_TIME);
        if (downForRecalTimeout.execute()) {
            gimbal->setPrevTargetPitch(SECOND_PITCH_OFFSET);
            drivers->recal.forceCalibration();
        }
    } else {
        downForRecalTimeout.stop();
    }
    
    if (mappingInput) {
        if(upForMappingTimeout.isStopped()) upForMappingTimeout.restart(MAPPING_HOLD_TIME);
#ifdef SENTRY
        // MappingCommand then takes the gimbal; it owns the save hold.
        if (upForMappingTimeout.execute()) drivers->mapping.requestStart();
#endif
    } else {
        upForMappingTimeout.stop();
    }
}

void JoystickMoveCommand::executeWith(src::Drivers* drivers, GimbalSubsystem* gimbal) {
    bool firstTime = false;
    bool isOffset = false;
    executeWith(drivers, gimbal, firstTime, isOffset);
}
void JoystickMoveCommand::executeWith(src::Drivers* drivers, GimbalSubsystem* gimbal, bool& firstTime, bool& isOffset) {
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