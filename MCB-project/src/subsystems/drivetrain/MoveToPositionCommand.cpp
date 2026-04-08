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
            targetVelocity = Pose2d(inputVelocity.getX(), inputVelocity.getY(), SPIN_VELO);
        else if(drivers->refSerial.getGameData().gameStage == RefSerialData::Rx::GameStage::COUNTDOWN){
            targetVelocity = Pose2d(0.0f, 0.0f, SPIN_VELO);
        }

        else
            targetVelocity = Pose2d(targetVelocity.getX(), targetVelocity.getY(), 0);
    }


    float referenceAngle = gimbal->getYawEncoderValue() - gimbal->getYawAngleRelativeWorld();


    // Vector2d targetPositionAdjusted = targetPosition.vec() + startPosition;
    currentPosition = Pose2d(drivers->i2c.odom.getX(), drivers->i2c.odom.getY(), referenceAngle);

  

    drivetrain->setTargetPosition(targetPosition, currentPosition, targetVelocity);
    // drivetrain->setTargetTranslation(drive, false);
}

bool MoveToPositionCommand::isFinished() const { return !drivers->remote.isConnected() || (targetPosition-currentPosition).magnitude() < tolerance; }


void MoveToPositionCommand::end(bool cancel) { 
    if(cancel)
        drivetrain->setTargetTranslation(targetVelocity, false);
}

}  // namespace commands