#include "AutoDriveCommand.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"
#include "util/Pose2d.hpp"

namespace commands {

void AutoDriveCommand::initialize() {
    targetPosition = Pose2d(0, 0, 0);
    targetVelocity = Pose2d(0, 0, 0);
}
void AutoDriveCommand::execute() {
    float referenceAngle = gimbal->getYawEncoderValue();
  
    int action = 0;
    jetson->updateROS(&targetPosition, &targetVelocity, &action);
       
    //TODO this is just wrong
    Pose2d drive = targetPosition;

    drivetrain->setTargetTranslation(drive, false);
}

bool AutoDriveCommand::isFinished() const { return !drivers->remote.isConnected(); }

void AutoDriveCommand::end(bool cancel) {}

}  // namespace commands