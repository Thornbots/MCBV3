#include "DrivetrainDriveCommand.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"
#include "util/Pose2d.hpp"

namespace commands {

void DrivetrainDriveCommand::initialize() {
    x = 0;
    y = 0;
    r = 0;
    autoBoost = false;
    
    drivetrain->isInControllerMode = controlMode == ControlMode::CONTROLLER;
    drivetrain->isInKeyboardMode = controlMode == ControlMode::KEYBOARD;
    drivetrain->isPeeking = driveMode == DriveMode::PEEK_LEFT || driveMode == DriveMode::PEEK_RIGHT;
    drivetrain->isPeekingLeft = driveMode == DriveMode::PEEK_LEFT;
    drivetrain->isBeyblading = driveMode == DriveMode::BEYBLADE;
}
void DrivetrainDriveCommand::execute() {
    float referenceAngle = gimbal->getYawEncoderValue();

    if (controlMode == ControlMode::KEYBOARD) {
        x = drivers->remote.keyPressed(Remote::Key::D) - drivers->remote.keyPressed(Remote::Key::A);
        y = drivers->remote.keyPressed(Remote::Key::W) - drivers->remote.keyPressed(Remote::Key::S);
        boost = drivers->remote.keyPressed(Remote::Key::SHIFT);

        int scroll = signum(drivers->remote.getMouseZ()); //mouse z is in increments of 10, making it -1, 0, or 1
        if(oldScroll!=scroll)
            drivetrain->linearVelocityMultiplierTimes100 += scroll * LINEAR_VELOCITY_INCREMENT_TIMES_100;
        oldScroll = scroll;

        if(drivers->remote.keyPressed(Remote::Key::V))
            drivetrain->linearVelocityMultiplierTimes100 = MAX_LINEAR_VELOCITY_TIMES_100;
            
        if(drivers->remote.keyPressed(Remote::Key::C))
            drivetrain->linearVelocityMultiplierTimes100 = MIN_LINEAR_VELOCITY_TIMES_100;
            
    } else if (controlMode == ControlMode::CONTROLLER) {
        x = drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL) * 3;
        y = drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL) * 3;
    } else {
        drivetrain->stopMotors();
        return;
    }
    
    // controller goes fast
    if(drivetrain->linearVelocityMultiplierTimes100==0)
        drivetrain->linearVelocityMultiplierTimes100 = MAX_LINEAR_VELOCITY_TIMES_100;
    //clamp
    if(drivetrain->linearVelocityMultiplierTimes100>MAX_LINEAR_VELOCITY_TIMES_100) 
        drivetrain->linearVelocityMultiplierTimes100 = MAX_LINEAR_VELOCITY_TIMES_100;
    else if(drivetrain->linearVelocityMultiplierTimes100<MIN_LINEAR_VELOCITY_TIMES_100) 
        drivetrain->linearVelocityMultiplierTimes100 = MIN_LINEAR_VELOCITY_TIMES_100;

    if (driveMode == DriveMode::NO_SPIN || !gimbal->isYawMotorOnline()) {
        autoBoost = false;
        r = 0;
        x *= MAX_NOSPIN_LINEAR_VELOCITY_TIMES_100 / 100.0f;
        y *= MAX_NOSPIN_LINEAR_VELOCITY_TIMES_100 / 100.0f;
    } else if (driveMode == DriveMode::BEYBLADE) {
        autoBoost = true;
        r = 19.5f * SPIN_DIRECTION;
        x *= drivetrain->linearVelocityMultiplierTimes100 / 100.0f;
        y *= drivetrain->linearVelocityMultiplierTimes100 / 100.0f;
    } else {
        autoBoost = false;
        x *= MAX_LINEAR_SPEED;
        y *= MAX_LINEAR_SPEED;
        float targetAngle = 0.0f;
        if (driveMode == DriveMode::PEEK_LEFT) {
            targetAngle = PEEK_LEFT_AMT;
        } else if (driveMode == DriveMode::PEEK_RIGHT) {
            targetAngle = PEEK_RIGHT_AMT;
        } 
        r = drivetrain->calculateRotationPID(targetAngle + referenceAngle); 
    }

    Pose2d drive(x, y, r);


    if (autoBoost == true && (drivetrain->angularVel < 6.0f || drivetrain->powerLimit >= 100.0f)){
        boost = true;

    }
    drivetrain->setTargetTranslation(drive.rotate(referenceAngle), boost);
}

bool DrivetrainDriveCommand::isFinished() const { return !drivers->remote.isConnected(); }

void DrivetrainDriveCommand::end(bool) {
    drivetrain->isInControllerMode = false;
    drivetrain->isInKeyboardMode = false;
    drivetrain->isBeyblading = false;
    drivetrain->isPeeking = false;
    drivetrain->isPeekingLeft = false;
}

}  // namespace commands