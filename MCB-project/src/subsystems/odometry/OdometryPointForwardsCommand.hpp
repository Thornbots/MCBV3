#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/odometry/OdometrySubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"

#include "drivers.hpp"

namespace commands
{
using subsystems::OdometrySubsystem;
using subsystems::GimbalSubsystem;
using tap::communication::serial::Remote;

class OdometryPointForwardsCommand : public tap::control::Command
{
public:
    OdometryPointForwardsCommand(src::Drivers* drivers, OdometrySubsystem* odometry, GimbalSubsystem* gimbal)
        : drivers(drivers),
          odometry(odometry), 
          gimbal(gimbal)
    {
        addSubsystemRequirement(odometry);
    }

    void initialize() override {
    };
    
    void execute() override {
        odoAngleRelativeWorld = gimbal->getYawAngleRelativeWorld() - gimbal->getYawEncoderValue() + odometry->getOdoEncoderValue();
        odoVelRelativeWorld = PI / 180 * drivers->bmi088.getGz() - gimbal->getYawVel() + odometry->getOdoVel();
        odometry->updateMotor(0, odoAngleRelativeWorld, odoVelRelativeWorld);
    };

    void end(bool interrupted) override {};

    bool isFinished() const {return false;};

    const char* getName() const override { return "point forwards odometry command"; }

private:
    src::Drivers* drivers;
    OdometrySubsystem* odometry;
    GimbalSubsystem* gimbal;

    float odoAngleRelativeWorld, odoVelRelativeWorld;
};
}  // namespace commands