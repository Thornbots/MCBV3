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
        : drivers(drivers), drive(drive), gimbal(gimbal), flywheel(flywheel), indexer(indexer),
          pointOdo(drivers, odo, gimbal) {
        addSubsystemRequirement(drive);
        addSubsystemRequirement(gimbal);
        addSubsystemRequirement(flywheel);
        addSubsystemRequirement(indexer);
        addSubsystemRequirement(odo);
    }

    void initialize() override {
        pointOdo.initialize();
        gimbal->clearBuildup();
        execute();
    }
    void execute() override {
        flywheel->setTargetVelocity(0);
        flywheel->stopMotors();
        indexer->stopIndex();
        const bool ready = drivers->mapping.getState() == src::MappingMachine::State::Ready;
        drivers->leds.set(tap::gpio::Leds::Green, ready);
        drivers->leds.set(tap::gpio::Leds::Red, false);
        drivers->leds.set(tap::gpio::Leds::Blue, false);
        if (drivers->mapping.readyBeep(tap::arch::clock::getTimeMilliseconds()))
            tap::buzzer::playNote(&drivers->pwm, 1000);
        else
            tap::buzzer::silenceBuzzer(&drivers->pwm);
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
        drivers->leds.set(tap::gpio::Leds::Green, false);
        tap::buzzer::silenceBuzzer(&drivers->pwm);
    }
    bool isFinished() const override { return !drivers->mapping.isActive(); }
    const char* getName() const override { return "MCB mapping restrictions"; }

private:
    src::Drivers* drivers;
    subsystems::DrivetrainSubsystem* drive;
    subsystems::GimbalSubsystem* gimbal;
    subsystems::FlywheelSubsystem* flywheel;
    subsystems::IndexerSubsystem* indexer;
    commands::OdometryPointForwardsCommand pointOdo;
};
}  // namespace robots
