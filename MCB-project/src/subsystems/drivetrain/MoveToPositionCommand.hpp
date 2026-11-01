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
        : targetPosition(targetPosition),
          drivetrain(drive),
          drivers(drivers),
          gimbal(gimbal),
          odo(odo),
          tolerance(tolerance){
        targetVelocity = Pose2d(targetVelocityInput.getX(), targetVelocityInput.getY(), MOVE_TO_POS_SPIN_VELO);
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    void setTolerance(float arrivalTolerance) { tolerance = arrivalTolerance; }

    const char* getName() const override { return "move to position command"; }
    DrivetrainSubsystem* getDrivetrain() {return drivetrain;}

    Pose2d targetPosition;
    Pose2d inputVelocity;

    DrivetrainSubsystem* drivetrain;
    static constexpr float MOVE_TO_POS_SPIN_VELO = -8.0;  //was 12.0   here tune spin velo
    // Replace the requested spin with a hold of the nearest field quarter turn, so the
    // square chassis passes openings narrower than its spin sweep.
    bool holdQuarterTurn = false;
protected:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;
    OdometrySubsystem* odo;

    float tolerance;
    Pose2d targetVelocity;
    Pose2d currentPosition;

};
}  // namespace commands
