#include "MoveToPositionCommand.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"
#include "util/Pose2d.hpp"

namespace commands {
using namespace tap::communication::serial;

void MoveToPositionCommand::initialize() {

}

void MoveToPositionCommand::execute() {

    if (drivers->refSerial.getRefSerialReceivingData() && 
       (drivers->refSerial.getGameData().gameType == RefSerialData::Rx::GameType::ROBOMASTER_RMUL_3V3)) {

        if(drivers->refSerial.getGameData().gameStage == RefSerialData::Rx::GameStage::IN_GAME)
        // targetVelocity = Pose2d(inputVelocity.getX(), inputVelocity.getY(), MOVE_TO_POS_SPIN_VELO);
            targetVelocity = inputVelocity; //let simpleautodrive control spin velo too
        else if(drivers->refSerial.getGameData().gameStage == RefSerialData::Rx::GameStage::COUNTDOWN){
            targetVelocity = Pose2d(0.0f, 0.0f, MOVE_TO_POS_SPIN_VELO);
        }

        else //not in a game
            targetVelocity = inputVelocity; //let simpleautodrive control spin velo too [this should be merged with IN_GAME case]
    } else //no ref? or no 3v3
            targetVelocity = inputVelocity; //let simpleautodrive control spin velo too [this should be merged with IN_GAME case]


    float referenceAngle = gimbal->getYawEncoderValue() - gimbal->getYawAngleRelativeWorld();


    // Vector2d targetPositionAdjusted = targetPosition.vec() + startPosition;
    currentPosition = Pose2d(odo->getX(), odo->getY(), referenceAngle);

  

    drivetrain->setTargetPosition(targetPosition, currentPosition, targetVelocity);
    // drivetrain->setTargetTranslation(drive, false);
}

bool MoveToPositionCommand::isFinished() const { return !drivers->remote.isConnected() || (targetPosition-currentPosition).magnitude() < tolerance; }


void MoveToPositionCommand::end(bool cancel) { 
    if(cancel)
        drivetrain->setTargetTranslation(targetVelocity, false);
}

}  // namespace commands