#pragma once

#include "tap/control/command.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"

#include "drivers.hpp"

namespace commands {
using subsystems::DrivetrainSubsystem;

class DrivetrainStopCommand : public tap::control::Command {
public:
    DrivetrainStopCommand(src::Drivers* drivers, DrivetrainSubsystem* drive) : drivers(drivers), drivetrain(drive) { addSubsystemRequirement(drive); }

    void initialize() override {};

    void execute() override {drivetrain->stopMotors();}

    void end(bool) override {};

    bool isFinished() const override {return false;};

    const char* getName() const override { return "drive stop command"; }

private:
    src::Drivers* drivers;
    DrivetrainSubsystem* drivetrain;
};
}  // namespace commands