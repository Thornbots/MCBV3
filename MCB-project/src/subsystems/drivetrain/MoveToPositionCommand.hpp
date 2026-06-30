#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/odometry/OdometrySubsystem.hpp"


#include "drivers.hpp"

namespace commands {
using subsystems::DrivetrainSubsystem;
using subsystems::GimbalSubsystem;
using subsystems::OdometrySubsystem;

using tap::communication::serial::Remote;


class MoveToPositionCommand : public tap::control::Command {
public:
    MoveToPositionCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal, OdometrySubsystem* odo, Pose2d targetPosition, Vector2d targetVelocityInput, float tolerance = 0.2f)
        : drivers(drivers),
          drivetrain(drive),
          gimbal(gimbal),
          odo(odo),
          tolerance(tolerance),
          targetPosition(targetPosition){
        targetVelocity = Pose2d(targetVelocityInput.getX(), targetVelocityInput.getY(), SPIN_VELO);
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "move to position command"; }
    DrivetrainSubsystem* getDrivetrain() {return drivetrain;}

    Pose2d targetPosition;
    Pose2d inputVelocity;

protected:
    src::Drivers* drivers;
    DrivetrainSubsystem* drivetrain;
    GimbalSubsystem* gimbal;
    OdometrySubsystem* odo;

    float tolerance;
    Pose2d targetVelocity;
    Pose2d currentPosition;

    static constexpr float SPIN_VELO = 6.0;  //12.0   here tune spin velo
};
}  // namespace commands