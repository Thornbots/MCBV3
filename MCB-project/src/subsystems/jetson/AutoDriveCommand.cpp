// TODO: ros-driven navigation is going likely going to change a lot

#include "AutoDriveCommand.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"
#include "util/Pose2d.hpp"

namespace commands {
using namespace tap::communication::serial;

int count = 0;
// Vector2d startPosition = Vector2d(0, 0);
void AutoDriveCommand::initialize() {
    isScheduled = true;

    count = 0;
    
    targetPosition = Vector2d(odo->getX(), odo->getY());
}

void AutoDriveCommand::execute() {
    targetVelocity = Pose2d(0, 0, 9);
    Vector2d jetsonExpectedPosition = Vector2d(0, 0);
    bool allowSpinning = true;
    bool allowMoving = true;

    if (drivers->refSerial.getRefSerialReceivingData() && 
       (drivers->refSerial.getGameData().gameType == RefSerialData::Rx::GameType::ROBOMASTER_RMUL_3V3)) {

        allowSpinning = false;
        allowMoving = false;

        if (drivers->refSerial.getGameData().gameStage == RefSerialData::Rx::GameStage::IN_GAME) {
            // allow both
            allowSpinning = true;
            allowMoving = true;
        }

        if (drivers->refSerial.getGameData().gameStage == RefSerialData::Rx::GameStage::COUNTDOWN) {
            // countdown, only allow spinning
            allowSpinning = true;
        }

    }
    // allowMoving = false;

    // minus the chassis' field heading: the IMU's zero is the start's heading
    float referenceAngle = gimbal->getYawEncoderValue() - gimbal->getYawAngleRelativeWorld() - odo->getStartYaw();

    // crude autodrive implementation
    // count++;

    // if (count > 400) {
    //     count = 0;
    // } else if (count > 200) {
    //     targetPosition = Pose2d(0.05, 0, 0);
    // } else {
    //     targetPosition = Pose2d(0, 0, 0);
    // }

    jetson->updateROS(&targetPosition, &targetVelocity, &jetsonExpectedPosition);


    Pose2d currentPosition = Pose2d(odo->getX() + offsetX, odo->getY() + offsetY, referenceAngle);
    
    float posX = targetPosition.getX();
    float posY = targetPosition.getY();
    float velX = targetVelocity.getX();
    float velY = targetVelocity.getY();
    float velR = targetVelocity.getRotation();
    if(!allowMoving){
        velX = 0;
        velY = 0;
        posX = currentPosition.getX();
        posY = currentPosition.getY();
    }
    if(!allowSpinning){
        velR = 0;
    }
    targetPosition = Vector2d{posX, posY};
    targetVelocity = Pose2d{velX, velY, velR};

    drivetrain->setTargetPosition(targetPosition, currentPosition, targetVelocity);
    // drivetrain->setTargetTranslation(drive, false);
}

bool AutoDriveCommand::isFinished() const { return !drivers->remote.isConnected(); }

bool AutoDriveCommand::getIsScheduled() { return isScheduled; }

void AutoDriveCommand::end(bool) {
    isScheduled = false;
}

}  // namespace commands