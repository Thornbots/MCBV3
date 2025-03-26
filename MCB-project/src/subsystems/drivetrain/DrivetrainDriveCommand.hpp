#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"

#include "drivers.hpp"

namespace commands {
using subsystems::DrivetrainSubsystem;
using subsystems::GimbalSubsystem;

using tap::communication::serial::Remote;

enum DriveMode { BEYBLADE, BEYBLADE2, FOLLOW_TURRET, PEEK_LEFT, PEEK_RIGHT };
enum ControlMode { KEYBOARD, CONTROLLER, DISABLED  };

class DrivetrainDriveCommand : public tap::control::Command {
public:
    DrivetrainDriveCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal, DriveMode driveMode, ControlMode controlMode)
        : drivers(drivers),
          drivetrain(drive),
          gimbal(gimbal),
          driveMode(driveMode),
          controlMode(controlMode) {
        addSubsystemRequirement(drive);
    }

    void initialize() override {};

    void execute() override;

    void end(bool interrupted) override {};

    bool isFinished() const { return false; };

    const char* getName() const override { return "drive command"; }

    void setDriveMode(DriveMode newDriveMode) { driveMode = newDriveMode; } 

    void setControlMode(ControlMode newControlMode) { controlMode = newControlMode; }

private:
    src::Drivers* drivers;
    DrivetrainSubsystem* drivetrain;
    GimbalSubsystem* gimbal;
    DriveMode driveMode;
    ControlMode controlMode;
    float x, y, r;
};
}  // namespace commands