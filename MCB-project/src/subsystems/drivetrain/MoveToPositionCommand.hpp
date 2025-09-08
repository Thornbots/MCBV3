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
    MoveToPositionCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal, Pose2d targetPosition, float tolerance = 0.2f)
        : drivers(drivers),
          drivetrain(drive),
          gimbal(gimbal),
          tolerance(tolerance),
          targetPosition(targetPosition){
        targetVelocity = Pose2d(0, 0, 10.5f);
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

    float tolerance;
    Pose2d targetPosition;
    Pose2d targetVelocity;
    Pose2d currentPosition;

};
}  // namespace commands