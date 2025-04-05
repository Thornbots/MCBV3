#pragma once

#include "tap/control/command.hpp"

#include "subsystems/servo/ServoSubsystem.hpp"

#include "drivers.hpp"

namespace commands {
using subsystems::ServoSubsystem;

class CloseServoCommand : public tap::control::Command {
public:
    CloseServoCommand(src::Drivers* drivers, ServoSubsystem* servo) : drivers(drivers), servo(servo) { addSubsystemRequirement(servo); }

    void initialize() override {};

    void execute() override {servo->setTargetPosition(ServoSubsystem::CLOSED_POSITION);};

    void end(bool interrupted) override {};

    bool isFinished() const override { return false; };

    const char* getName() const override { return "close servo command"; }

private:
    src::Drivers* drivers;
    ServoSubsystem* servo;
};
}  // namespace commands