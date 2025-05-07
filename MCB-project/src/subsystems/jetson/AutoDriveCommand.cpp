#include "AutoDriveCommand.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"
#include "util/Pose2d.hpp"

namespace commands {

void AutoDriveCommand::initialize() {
    targetPosition = Pose2d(0, 0, 0);
    targetVelocity = Pose2d(0, 0, 8);
}
void AutoDriveCommand::execute() {
    float referenceAngle = gimbal->getYawEncoderValue() - gimbal->getYawAngleRelativeWorld();
  
    int action = 0;
    jetson->updateROS(&targetPosition, &targetVelocity, &action);

    Pose2d currentPosition = Pose2d(drivers->i2c.odom.getX(), drivers->i2c.odom.getY(), referenceAngle);

    drivetrain->setTargetPosition(targetPosition.vec(), currentPosition, targetVelocity);
    // drivetrain->setTargetTranslation(drive, false);
}

bool AutoDriveCommand::isFinished() const { return !drivers->remote.isConnected(); }

void AutoDriveCommand::end(bool cancel) {}

}  // namespace commands