#pragma once

#include "tap/control/command.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/odometry/OdometrySubsystem.hpp"

#include "drivers.hpp"

namespace commands {
using subsystems::DrivetrainSubsystem;

class DrivetrainStopCommand : public tap::control::Command {
public:
    DrivetrainStopCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, subsystems::GimbalSubsystem* gimbal = nullptr,
        subsystems::OdometrySubsystem* odometry = nullptr)
        : drivers(drivers), drivetrain(drive), gimbal(gimbal), odometry(odometry) { addSubsystemRequirement(drive); }

    void initialize() override {};

    void execute() override {
        using Ref = RefSerialData;
        const bool powered = static_cast<bool>(drivers->refSerial.getRobotData().robotPower &
            Ref::Rx::RobotPower::CHASSIS_HAS_POWER);
        if (powered && drivers->refSerial.getGameData().gameStage == Ref::Rx::GameStage::IN_GAME) {
            if (gimbal && odometry) {
                const float heading = gimbal->getYawEncoderValue() - gimbal->getYawAngleRelativeWorld() -
                                      odometry->getStartYaw();
                drivetrain->holdRawPosition({odometry->getAimX(), odometry->getAimY(), heading});
            } else drivetrain->setTargetTranslation(Pose2d{}, false);
        } else {
            drivetrain->stopMotors();
        }
    }

    void end(bool) override {};

    bool isFinished() const override {return false;};

    const char* getName() const override { return "drive stop command"; }

private:
    src::Drivers* drivers;
    DrivetrainSubsystem* drivetrain;
    subsystems::GimbalSubsystem* gimbal;
    subsystems::OdometrySubsystem* odometry;
};
}  // namespace commands