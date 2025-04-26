#include "DrivetrainDriveCommand.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"
#include "util/Pose2d.hpp"

namespace commands {

void DrivetrainDriveCommand::initialize() {
    x = 0;
    y = 0;
    r = 0;
}
void DrivetrainDriveCommand::execute() {
    float referenceAngle = gimbal->getYawEncoderValue();

    if (controlMode == ControlMode::KEYBOARD) {
        x = drivers->remote.keyPressed(Remote::Key::D) - drivers->remote.keyPressed(Remote::Key::A);
        y = drivers->remote.keyPressed(Remote::Key::W) - drivers->remote.keyPressed(Remote::Key::S);
        boost = drivers->remote.keyPressed(Remote::Key::SHIFT);

    } else if (controlMode == ControlMode::CONTROLLER) {
        x = drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
        y = drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL);
    } else {
        drivetrain->stopMotors();
        return;
    }

    if (driveMode == DriveMode::BEYBLADE) {
        r = 10.5f;
        x *= 1.75f;
        y *= 1.75f;
    } else if (driveMode == DriveMode::BEYBLADE2) {
        r = 10.5f;
    } else if (driveMode == DriveMode::NO_SPIN) {
        r = 0;

    } else {
        float targetAngle = 0.0f;
        if (driveMode == DriveMode::PEEK_LEFT) {
            targetAngle = PEEK_LEFT_AMT;

        } else if (driveMode == DriveMode::PEEK_RIGHT) {
            targetAngle = PEEK_RIGHT_AMT;

        } else {
            x *= 2.5f;
            y *= 2.5f;
        }
        r = drivetrain->calculateRotationPID(targetAngle + referenceAngle); 
    }

    Pose2d drive(x, y, r);

    drivetrain->setTargetTranslation(drive.rotate(referenceAngle), (bool)boost);
}

bool DrivetrainDriveCommand::isFinished() const { return !drivers->remote.isConnected(); }

void DrivetrainDriveCommand::end(bool cancel) {}

}  // namespace commands