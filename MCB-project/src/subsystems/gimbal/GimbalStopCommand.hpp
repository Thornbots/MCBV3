#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"

#include "drivers.hpp"

namespace commands
{
using subsystems::GimbalSubsystem;
using tap::communication::serial::Remote;

class GimbalStopCommand : public tap::control::Command
{
public:
    GimbalStopCommand(src::Drivers* drivers, GimbalSubsystem* gimbal)
        : drivers(drivers),
          gimbal(gimbal)
    {
        addSubsystemRequirement(gimbal);
    }

    void initialize() override {};

    void execute() override {gimbal->stopMotors();};

    void end(bool) override {};

    bool isFinished() const {return false;};

    const char* getName() const override { return "stop gimbal command"; }

private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;
};
}  // namespace commands