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


class AutoDriveCommand : public tap::control::Command {
public:
    AutoDriveCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal, JetsonSubsystem* jetson)
        : drivers(drivers),
          jetson(jetson),
          drivetrain(drive),
          gimbal(gimbal) {
        addSubsystemRequirement(drive);
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "auto drive command"; }


private:
    src::Drivers* drivers;
    JetsonSubsystem* jetson;
    DrivetrainSubsystem* drivetrain;
    GimbalSubsystem* gimbal;

    Pose2d targetPosition;
    Pose2d targetVelocity;
};
}  // namespace commands