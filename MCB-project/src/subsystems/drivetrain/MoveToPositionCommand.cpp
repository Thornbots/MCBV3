#include "MoveToPositionCommand.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"
#include "util/Pose2d.hpp"
#include "util/RefereeMatch.hpp"

namespace commands {
using namespace tap::communication::serial;

void MoveToPositionCommand::initialize() {

}

void MoveToPositionCommand::execute() {
    if (!odo->isNavigationLocalized()) {
        drivetrain->stopMotors();
        return;
    }


    // Countdown holds position and spins; otherwise SimpleAutoDrive sets translation and spin.
    if (referee_match::rmul3v3StageIs(drivers->refSerial, RefSerialData::Rx::GameStage::COUNTDOWN))
        targetVelocity = Pose2d(0.0f, 0.0f, MOVE_TO_POS_SPIN_VELO);
    else
        targetVelocity = inputVelocity;


    // minus the chassis' field heading: the IMU's zero is the start's heading
    float referenceAngle = gimbal->getYawEncoderValue() - gimbal->getYawAngleRelativeWorld() - odo->getStartYaw();


    // Vector2d targetPositionAdjusted = targetPosition.vec() + startPosition;
    currentPosition = Pose2d(odo->getX(), odo->getY(), referenceAngle);

  

    drivetrain->setTargetPosition(targetPosition, currentPosition, targetVelocity);
    // drivetrain->setTargetTranslation(drive, false);
}

bool MoveToPositionCommand::isFinished() const { return !drivers->remote.isConnected() || (odo->isNavigationLocalized() && (targetPosition-currentPosition).magnitude() < tolerance); }


void MoveToPositionCommand::end(bool cancel) {
    if (!odo->isNavigationLocalized()) {
        drivetrain->stopMotors();
        return;
    }
    if(cancel)
        drivetrain->setTargetTranslation(targetVelocity, false);
}

}  // namespace commands