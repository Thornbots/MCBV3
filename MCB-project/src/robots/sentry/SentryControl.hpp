#include "robots/RobotControl.hpp"
#include "robots/sentry/SentryHardware.hpp"

#include "subsystems/drivetrain/DrivetrainDriveCommand.hpp"
#include "subsystems/drivetrain/DrivetrainStopCommand.hpp"
#include "subsystems/flywheel/ShooterStartCommand.hpp"
#include "subsystems/flywheel/ShooterStopCommand.hpp"
#include "subsystems/gimbal/JoystickMoveCommand.hpp"
#include "subsystems/gimbal/MouseMoveCommand.hpp"
#include "subsystems/gimbal/GimbalStopCommand.hpp"
#include "subsystems/jetson/AutoAimAndFireCommand.hpp"
#include "subsystems/indexer/IndexerNBallsCommand.hpp"
#include "subsystems/indexer/IndexerUnjamCommand.hpp"
#include "subsystems/indexer/IndexerStopCommand.hpp"
#include "subsystems/odometry/OdometryStopCommand.hpp"
#include "subsystems/jetson/AutoDriveCommand.hpp"
#include "subsystems/odometry/OdometryPointForwardsCommand.hpp"
#include "subsystems/indexer/DoubleIndexerSubsystem.hpp"
#include "util/trigger.hpp"

#include "subsystems/jetson/JetsonSubsystem.hpp"

#include "drivers.hpp"

namespace robots {
class SentryControl : public ControlInterface {
public:
    // pass drivers back to root robotcontrol to store
    SentryControl(src::Drivers* drivers) : drivers(drivers), hardware(SentryHardware{drivers}) {}
    // functions we are using
    void initialize() override {
        jetson.initialize();

        // Initialize subsystems
        gimbal.initialize();
        flywheel.initialize();
        indexer.initialize();
        drivetrain.initialize();
        odo.initialize();

        // Run startup commands
        gimbal.setDefaultCommand(&stopGimbal);
        flywheel.setDefaultCommand(&shooterStop);
        drivetrain.setDefaultCommand(&stopDriveCommand);
        indexer.setDefaultCommand(&indexerStopCommand);
        odo.setDefaultCommand(&odoStop);

        shootButton.onTrue(&shooterStart)->whileTrue(&indexer10Hz);
        unjamButton.onTrue(&shooterStop)->whileTrue(&indexerUnjam);

        autoTrigger.whileTrue(&autoFireCommand)->onFalse(&lookJoystick)->whileTrue(&shooterStart);
        autoDriveTrigger.whileTrue(&autoDriveCommand)->onTrue(&odoPointForwards);
        // drive commands 

        joystickDrive0.onTrue(&noSpinDriveCommand)->onTrue(&lookJoystick)->onTrue(&odoPointForwards);
        joystickDrive1.onTrue(&drivetrainFollowJoystick)->onTrue(&lookJoystick)->onTrue(&odoPointForwards);
        joystickDrive2.onTrue(&beybladeJoystick)->onTrue(&lookJoystick)->onTrue(&odoPointForwards);
    }

    void update() override {
        for (Trigger* trigger : triggers) {
            trigger->update();
        }
    }

    src::Drivers* drivers;
    SentryHardware hardware;

    // subsystems
    subsystems::GimbalSubsystem gimbal{drivers, &hardware.yawMotor, &hardware.pitchMotor};
    subsystems::FlywheelSubsystem flywheel{drivers, &hardware.flywheelMotor1, &hardware.flywheelMotor2};
    subsystems::DoubleIndexerSubsystem indexer{drivers, &hardware.indexMotor1, &hardware.indexMotor2};
    subsystems::DrivetrainSubsystem drivetrain{drivers, &hardware.driveMotor1, &hardware.driveMotor2, &hardware.driveMotor3, &hardware.driveMotor4};
    subsystems::JetsonSubsystem jetson{drivers};
    subsystems::OdometrySubsystem odo{drivers, &hardware.odoMotor};

    // commands
    commands::JoystickMoveCommand lookJoystick{drivers, &gimbal};
    commands::GimbalStopCommand stopGimbal{drivers, &gimbal};
    commands::AutoAimAndFireCommand autoFireCommand{drivers, &gimbal, &indexer, &jetson};

    commands::ShooterStartCommand shooterStart{drivers, &flywheel};
    commands::ShooterStopCommand shooterStop{drivers, &flywheel};

    commands::IndexerNBallsCommand indexer10Hz{drivers, &indexer, -1, 10};
    commands::IndexerUnjamCommand indexerUnjam{drivers, &indexer};

    commands::IndexerStopCommand indexerStopCommand{drivers, &indexer};

    commands::OdometryPointForwardsCommand odoPointForwards{drivers, &odo, &gimbal};
    commands::OdometryStopCommand odoStop{drivers, &odo};

    // CHANGE NUMBERS LATER
    commands::DrivetrainDriveCommand drivetrainFollowJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::FOLLOW_TURRET, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand beybladeJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE2, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand noSpinDriveCommand{drivers, &drivetrain, &gimbal, commands::DriveMode::NO_SPIN, commands::ControlMode::CONTROLLER};
    commands::AutoDriveCommand autoDriveCommand{drivers, &drivetrain, &gimbal, &jetson};

    commands::DrivetrainStopCommand stopDriveCommand{drivers, &drivetrain};

    // mappings 

    // shooting
    Trigger shootButton{drivers, Remote::Channel::WHEEL, -0.5};
    Trigger unjamButton{drivers, Remote::Channel::WHEEL, 0.5};

    // controller driving
    Trigger joystickDrive0{
        drivers,
        Remote::Switch::RIGHT_SWITCH,
        Remote::SwitchState::UP};  // = (Trigger(drivers, Remote::Key::Q) & Trigger(drivers, Remote::Key::E)) | Trigger(drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
    Trigger joystickDrive1{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID};
    Trigger joystickDrive2{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN};

    Trigger autoTrigger{drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP};
    Trigger autoDriveTrigger{drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN};


    Trigger* triggers[7] = {&joystickDrive0, &joystickDrive1, &joystickDrive2, &shootButton, &unjamButton, &autoTrigger, &autoDriveTrigger};  //, &indexSpinButton};
};
}  // namespace robots
