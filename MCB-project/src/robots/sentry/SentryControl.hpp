#include "robots/RobotControl.hpp"
#include "robots/sentry/SentryHardware.hpp"

#include "subsystems/drivetrain/DrivetrainDriveCommand.hpp"
#include "subsystems/drivetrain/DrivetrainStopCommand.hpp"
#include "subsystems/flywheel/ShooterStartCommand.hpp"
#include "subsystems/flywheel/ShooterStopCommand.hpp"
#include "subsystems/gimbal/JoystickMoveCommand.hpp"
#include "subsystems/gimbal/MouseMoveCommand.hpp"
#include "subsystems/gimbal/GimbalStopCommand.hpp"
#include "subsystems/indexer/IndexerNBallsCommand.hpp"
#include "subsystems/indexer/IndexerUnjamCommand.hpp"
#include "subsystems/indexer/IndexerStopCommand.hpp"
#include "util/trigger.hpp"

#include "drivers.hpp"

namespace robots {
class SentryControl : public ControlInterface {
public:
    // pass drivers back to root robotcontrol to store
    SentryControl(src::Drivers* drivers) : drivers(drivers), hardware(SentryHardware{drivers}) {}
    // functions we are using
    void initialize() override {
        // Initialize subsystems
        gimbal.initialize();
        flywheel.initialize();
        indexer.initialize();
        drivetrain.initialize();

        // Run startup commands
        gimbal.setDefaultCommand(&stopGimbal);
        flywheel.setDefaultCommand(&shooterStop);
        drivetrain.setDefaultCommand(&stopDriveCommand);
        indexer.setDefaultCommand(&indexerStopCommand);

        shootButton.onTrue(&shooterStart)->whileTrue(&indexer10Hz);
        unjamButton.onTrue(&shooterStop)->whileTrue(&indexerUnjam);


        // drive commands 

        joystickDrive0.onTrue(&noSpinDriveCommand)->onTrue(&lookJoystick);
        joystickDrive1.onTrue(&drivetrainFollowJoystick)->onTrue(&lookJoystick);
        joystickDrive2.onTrue(&beybladeJoystick)->onTrue(&lookJoystick);
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

    // commands old
    //  commands::JoystickMoveCommand look{drivers, &gimbal};
    //  commands::MouseMoveCommand look2{drivers, &gimbal};

    // commands::ShooterStartCommand shooterStart{drivers, &flywheel};
    // commands::ShooterStopCommand shooterStop{drivers, &flywheel};

    // commands::IndexerNBallsCommand indexer10Hz{drivers, &indexer, -1, 10};
    // commands::IndexerUnjamCommand indexerUnjam{drivers, &indexer};

    // commands from infantrycontrol
    commands::JoystickMoveCommand lookJoystick{drivers, &gimbal};
    commands::GimbalStopCommand stopGimbal{drivers, &gimbal};

    commands::ShooterStartCommand shooterStart{drivers, &flywheel};
    commands::ShooterStopCommand shooterStop{drivers, &flywheel};

    commands::IndexerNBallsCommand indexer10Hz{drivers, &indexer, -1, 10};
    commands::IndexerUnjamCommand indexerUnjam{drivers, &indexer};

    commands::IndexerStopCommand indexerStopCommand{drivers, &indexer};

    // CHANGE NUMBERS LATER
    commands::DrivetrainDriveCommand peekRight{drivers, &drivetrain, &gimbal, commands::DriveMode::PEEK_RIGHT, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand peekLeft{drivers, &drivetrain, &gimbal, commands::DriveMode::PEEK_LEFT, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand drivetrainFollowKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::FOLLOW_TURRET, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand drivetrainFollowJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::FOLLOW_TURRET, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand beybladeJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE2, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand beybladeSlowKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand beybladeFastKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE2, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand noSpinDriveCommand{drivers, &drivetrain, &gimbal, commands::DriveMode::NO_SPIN, commands::ControlMode::CONTROLLER};

    commands::DrivetrainStopCommand stopDriveCommand{drivers, &drivetrain};

    // mappings old
    //  HoldCommandMapping startShootMapping {
    //      drivers,
    //      {&indexer10Hz},
    //      RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP)};

    // HoldCommandMapping idleShootMapping {
    //     drivers,
    //     {&shooterStart},
    //     RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP)};

    // HoldCommandMapping stopShootMapping {
    //     drivers,
    //     {&indexerUnjam, &shooterStop},
    //     RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN)};

    // HoldCommandMapping startShootMappingMouse {
    //     drivers,
    //     {&shooterStart, &indexer10Hz},
    //     RemoteMapState(RemoteMapState::MouseButton::LEFT)
    // };

    // HoldCommandMapping stopShootMappingMouse {
    //     drivers,
    //     {&shooterStop, &indexerUnjam},
    //     RemoteMapState(RemoteMapState::MouseButton::LEFT)
    // };

    // mappings from infantrycontrol

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

    Trigger* triggers[5] = {&joystickDrive0, &joystickDrive1, &joystickDrive2, &shootButton, &unjamButton};  //, &indexSpinButton};
};
}  // namespace robots
