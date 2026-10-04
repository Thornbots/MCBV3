#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"

#include "drivers.hpp"

namespace commands
{
using subsystems::GimbalSubsystem;
using tap::communication::serial::Remote;

class MouseMoveCommand : public tap::control::Command
{
public:
    MouseMoveCommand(src::Drivers* drivers, GimbalSubsystem* gimbal)
        : drivers(drivers),
          gimbal(gimbal)
    {
        addSubsystemRequirement(gimbal);
    }

    void initialize() override;

    void execute() override;
    
    static void executeWith(src::Drivers* drivers, GimbalSubsystem* gimbal);

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "move turret mouse command"; }

protected:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;

};
}  // namespace commands