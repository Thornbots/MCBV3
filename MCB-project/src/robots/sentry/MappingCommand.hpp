// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "robots/sentry/MappingDrive.hpp"
#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/flywheel/FlywheelSubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/indexer/IndexerSubsystem.hpp"
#include "subsystems/odometry/OdometryPointForwardsCommand.hpp"

namespace robots {
class MappingCommand : public tap::control::Command {
public:
    MappingCommand(src::Drivers* drivers, subsystems::DrivetrainSubsystem* drive,
                   subsystems::GimbalSubsystem* gimbal, subsystems::FlywheelSubsystem* flywheel,
                   subsystems::IndexerSubsystem* indexer, subsystems::OdometrySubsystem* odo)
        : drivers(drivers), drive(drive), gimbal(gimbal), flywheel(flywheel), indexer(indexer), odo(odo),
          pointOdo(drivers, odo, gimbal) {
        addSubsystemRequirement(drive);
        addSubsystemRequirement(gimbal);
        addSubsystemRequirement(flywheel);
        addSubsystemRequirement(indexer);
        addSubsystemRequirement(odo);
    }

    void initialize() override {
        odo->stopMotors();
        gimbal->clearBuildup();
        execute();
    }
    void execute() override {
        flywheel->setTargetVelocity(0);
        flywheel->stopMotors();
        indexer->stopIndex();
        using Remote = tap::communication::serial::Remote;
        if (drivers->mapping.canDrive() && drivers->remote.isConnected()) {
            drive->setTargetTranslation(mappingDrive(
                drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL),
                drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL),
                -drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL), true), false);
        } else {
            drive->stopMotors();
        }
        // Encoder yaw zero faces the chassis front; world yaw follows chassis turns.
        gimbal->setAngles(drivers->bmi088.getYaw() - gimbal->getYawEncoderValue(), 0);
        pointOdo.execute();
    }
    void end(bool) override {
        drive->stopMotors();
        flywheel->setTargetVelocity(0);
        flywheel->stopMotors();
        indexer->stopIndex();
        gimbal->stopMotors();
    }
    bool isFinished() const override { return !drivers->mapping.isActive(); }
    const char* getName() const override { return "MCB mapping restrictions"; }

private:
    src::Drivers* drivers;
    subsystems::DrivetrainSubsystem* drive;
    subsystems::GimbalSubsystem* gimbal;
    subsystems::FlywheelSubsystem* flywheel;
    subsystems::IndexerSubsystem* indexer;
    subsystems::OdometrySubsystem* odo;
    commands::OdometryPointForwardsCommand pointOdo;
};
}  // namespace robots
