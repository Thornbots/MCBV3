#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/jetson/JetsonSubsystem.hpp"


#include "drivers.hpp"

namespace commands {
using subsystems::DrivetrainSubsystem;
using subsystems::GimbalSubsystem;
using subsystems::JetsonSubsystem;

using tap::communication::serial::Remote;


class MoveToPositionCommand : public tap::control::Command {
public:
    MoveToPositionCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal, Pose2d targetPosition)
        : drivers(drivers),
          drivetrain(drive),
          gimbal(gimbal),
          targetPosition(targetPosition){
        targetVelocity = Pose2d(0, 0, 5);
        addSubsystemRequirement(drive);
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "move to position command"; }


private:
    src::Drivers* drivers;
    DrivetrainSubsystem* drivetrain;
    GimbalSubsystem* gimbal;

    Pose2d targetPosition;
    Pose2d targetVelocity;
    Pose2d currentPosition;

    const float POS_ERROR_THRESHOLD = 0.2f;  // meters

};
}  // namespace commands