#include "AutoDriveCommand.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"
#include "util/Pose2d.hpp"

namespace commands {
int count = 0;
Vector2d startPosition = Vector2d(0, 0);
void AutoDriveCommand::initialize() {
    count = 0;
    targetPosition = Pose2d(0, 0, 0);
    targetVelocity = Pose2d(0, 0, 8);
    startPosition = Vector2d(drivers->i2c.odom.getX(), drivers->i2c.odom.getY());
}

void AutoDriveCommand::execute() {
    float referenceAngle = gimbal->getYawEncoderValue() - gimbal->getYawAngleRelativeWorld();
  
    int action = 0;
    count++;

    if (count > 400){
        count = 0;
    } else if (count > 200){
        targetPosition = Pose2d(0.05, 0, 0);
    } else {
        targetPosition = Pose2d(0, 0, 0);
    }

    // jetson->updateROS(&targetPosition, &targetVelocity, &action);

    Pose2d currentPosition = Pose2d(drivers->i2c.odom.getX(), drivers->i2c.odom.getY(), referenceAngle);

    drivetrain->setTargetPosition(targetPosition.vec() + startPosition, currentPosition, targetVelocity);
    // drivetrain->setTargetTranslation(drive, false);
}

bool AutoDriveCommand::isFinished() const { return !drivers->remote.isConnected(); }

void AutoDriveCommand::end(bool cancel) {}

}  // namespace commands