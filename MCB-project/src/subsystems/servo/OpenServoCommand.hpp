#pragma once

#include "tap/control/command.hpp"

#include "subsystems/servo/ServoSubsystem.hpp"

#include "drivers.hpp"

namespace commands {
using subsystems::ServoSubsystem;

class OpenServoCommand : public tap::control::Command {
public:
    OpenServoCommand(src::Drivers* drivers, ServoSubsystem* servo) : drivers(drivers), servo(servo) { addSubsystemRequirement(servo); }

    void initialize() override {};

    void execute() override {servo->setOpen();};

    void end(bool) override {};

    bool isFinished() const override { return false; };

    const char* getName() const override { return "open servo command"; }

private:
    src::Drivers* drivers;
    ServoSubsystem* servo;
};
}  // namespace commands