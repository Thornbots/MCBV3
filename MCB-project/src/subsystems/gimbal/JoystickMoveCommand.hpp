#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"

#include "drivers.hpp"

namespace commands
{
using subsystems::GimbalSubsystem;
using tap::communication::serial::Remote;

class JoystickMoveCommand : public tap::control::Command
{
public:
    JoystickMoveCommand(src::Drivers* drivers, GimbalSubsystem* gimbal)
        : drivers(drivers),
          gimbal(gimbal)
    {
        addSubsystemRequirement(gimbal);
    }

    JoystickMoveCommand(src::Drivers* drivers, GimbalSubsystem* gimbal, bool isOffset)
        : drivers(drivers),
          gimbal(gimbal),
          isOffset(isOffset)
    {
        addSubsystemRequirement(gimbal);
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "move turret joystick command"; }



private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;

    bool isCalibrated = false;

    float yaw = 0.0f, pitch = 0.0f;

    bool isOffset = false;
};
}  // namespace commands