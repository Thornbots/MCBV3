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

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "move turret mouse command"; }
    


private:
    bool isCalibrated = false;

protected:
    GimbalSubsystem* gimbal;
    src::Drivers* drivers;
    float yaw = 0.0f, pitch = 0.0f;
    float yawvel = 0.0f, pitchvel = 0.0f;

};
}  // namespace commands