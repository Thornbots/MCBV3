#include "AutoDriveCommand.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"
#include "util/Pose2d.hpp"

namespace commands {
using namespace tap::communication::serial;

int count = 0;
Vector2d startPosition = Vector2d(0, 0);
void AutoDriveCommand::initialize() {
    isScheduled = true;

    count = 0;
    targetPosition = Pose2d(0, 0, 0);
    startPosition = Vector2d(drivers->i2c.odom.getX(), drivers->i2c.odom.getY());

    drivers->leds.set(tap::gpio::Leds::Blue, true);
}

void AutoDriveCommand::execute() {
    targetVelocity = Pose2d(0, 0, 9);
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

    float referenceAngle = gimbal->getYawEncoderValue() - gimbal->getYawAngleRelativeWorld();

    int action = 0;
    count++;

    if (count > 400) {
        count = 0;
    } else if (count > 200) {
        targetPosition = Pose2d(0.05, 0, 0);
    } else {
        targetPosition = Pose2d(0, 0, 0);
    }

    // jetson->updateROS(&targetPosition, &targetVelocity, &action);

    Vector2d targetPositionAdjusted = targetPosition.vec() + startPosition;
    Pose2d currentPosition = Pose2d(drivers->i2c.odom.getX(), drivers->i2c.odom.getY(), referenceAngle);

    float posX = targetPositionAdjusted.getX();
    float posY = targetPositionAdjusted.getY();
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
    targetPositionAdjusted = Vector2d{posX, posY};
    targetVelocity = Pose2d{velX, velY, velR};

    drivetrain->setTargetPosition(targetPositionAdjusted, currentPosition, targetVelocity);
    // drivetrain->setTargetTranslation(drive, false);
}

bool AutoDriveCommand::isFinished() const { return !drivers->remote.isConnected(); }

bool AutoDriveCommand::getIsScheduled() { return isScheduled; }

void AutoDriveCommand::end(bool cancel) { 
    drivers->leds.set(tap::gpio::Leds::Blue, false);
    isScheduled = false;
}

}  // namespace commands