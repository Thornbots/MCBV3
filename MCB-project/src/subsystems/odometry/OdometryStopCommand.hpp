#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/odometry/OdometrySubsystem.hpp"

#include "drivers.hpp"

namespace commands
{
using subsystems::OdometrySubsystem;
using tap::communication::serial::Remote;

class OdometryStopCommand : public tap::control::Command
{
public:
    OdometryStopCommand(src::Drivers* drivers, OdometrySubsystem* odometry)
        : drivers(drivers),
          odometry(odometry)
    {
        addSubsystemRequirement(odometry);
    }

    void initialize() override {};

    void execute() override {odometry->stopMotors();};

    void end(bool interrupted) override {};

    bool isFinished() const {return false;};

    const char* getName() const override { return "stop odometry command"; }

private:
    src::Drivers* drivers;
    OdometrySubsystem* odometry;
};
}  // namespace commands