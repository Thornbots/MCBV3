#include "MoveToPositionCommand.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"
#include "util/Pose2d.hpp"

namespace commands {
using namespace tap::communication::serial;

void MoveToPositionCommand::initialize() {

}

void MoveToPositionCommand::execute() {

    float referenceAngle = gimbal->getYawEncoderValue() - gimbal->getYawAngleRelativeWorld();


    // Vector2d targetPositionAdjusted = targetPosition.vec() + startPosition;
    currentPosition = Pose2d(drivers->i2c.odom.getX(), drivers->i2c.odom.getY(), referenceAngle);

  

    drivetrain->setTargetPosition(targetPosition, currentPosition, targetVelocity);
    // drivetrain->setTargetTranslation(drive, false);
}

bool MoveToPositionCommand::isFinished() const { return !drivers->remote.isConnected() || (targetPosition-currentPosition).magnitude() < tolerance; }


void MoveToPositionCommand::end(bool cancel) { 
    drivers->leds.set(tap::gpio::Leds::Blue, false);
   if(cancel)  
        drivetrain->setTargetTranslation(targetVelocity, false);
}

}  // namespace commands