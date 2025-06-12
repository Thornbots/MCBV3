#include "robots/RobotControl.hpp"

#include "robots/hero/HeroHardware.hpp"

#include "subsystems/ui/UISubsystem.hpp"
#include "subsystems/ui/HeroDrawCommand.hpp"

#include "subsystems/indexer/HeroIndexerSubsystem.hpp"

#include "subsystems/gimbal/JoystickMoveCommand.hpp"
#include "subsystems/gimbal/MouseMoveCommand.hpp"

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
#include "subsystems/indexer/IndexerLoadCommand.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/trigger.hpp"

#include "drivers.hpp"

namespace robots {
class HeroControl : public ControlInterface {
public:
    // pass drivers back to root robotcontrol to store
    HeroControl(src::Drivers *drivers) : drivers(drivers), hardware(HeroHardware{drivers}) {}
    // functions we are using
    void initialize() override {
        // Initialize subsystems
        gimbal.initialize();
        flywheel.initialize();
        indexer.initialize(); //geometry issue or something, it physically doesn't work right now
        drivetrain.initialize();
        ui.initialize();
        
        // Run startup commands
        gimbal.setDefaultCommand(&stopGimbal);
        flywheel.setDefaultCommand(&shooterStop);
        drivetrain.setDefaultCommand(&stopDriveCommand);
        indexer.setDefaultCommand(&indexerLoadCommand);
        

        // Mouse and Keyboard mappings
        unjamKey.whileTrue(&indexerUnjam)->onTrue(&shooterStop)->onFalse(&indexerLoadCommand);
        shootKey.onTrue(&indexer1Hz)->onTrue(&shooterStart)->onFalse(&indexerLoadCommand);
        unjamButton.whileTrue(&indexerUnjam)->onTrue(&shooterStop)->onFalse(&indexerLoadCommand);
        shootButton.whileTrue(&indexer1Hz)->onTrue(&shooterStart)->onFalse(&indexerLoadCommand);
        //autoAimKey./*whileTrue(&autoCommand)->*/onFalse(&lookMouse)->whileTrue(&shooterStart);
        // implement speed mode

        toggleUIKey.onTrue(&draw)->onTrue(&drivetrainFollowKeyboard)->onTrue(&lookMouse); //press g to start robot
        drivers->commandScheduler.addCommand(&draw);
   
        // drive commands and also enable mouse looking

        peekLeftButton.onTrue(&peekLeft)->onFalse(&beybladeSlowKeyboard);
        peekRightButton.onTrue(&peekRight)->onFalse(&beybladeSlowKeyboard);

        beybladeType0Key.onTrue(&drivetrainFollowKeyboard)->onTrue(&lookMouse);
        beybladeType1Key.onTrue(&beybladeSlowKeyboard)->onTrue(&lookMouse);
        beybladeType2Key.onTrue(&beybladeFastKeyboard)->onTrue(&lookMouse);
 
        joystickDrive0.onTrue(&noSpinDriveCommand)->onTrue(&lookJoystick);
        joystickDrive1.onTrue(&drivetrainFollowJoystick)->onTrue(&lookJoystick);
        joystickDrive2.onTrue(&beybladeJoystick)->onTrue(&lookJoystick);
    }

    void update() override {

        for (Trigger* trigger : triggers) {
            trigger->update();
        }
    }

    src::Drivers *drivers;
    HeroHardware hardware;

    // Subsystems
    subsystems::UISubsystem ui{drivers};
    subsystems::GimbalSubsystem gimbal{drivers, &hardware.yawMotor, &hardware.pitchMotor};
    subsystems::FlywheelSubsystem flywheel{drivers, &hardware.flywheelMotor1, &hardware.flywheelMotor2};
    subsystems::HeroIndexerSubsystem indexer{drivers, &hardware.indexTopMotor, &hardware.indexBottomMotor};
    subsystems::DrivetrainSubsystem drivetrain{drivers, &hardware.driveMotor1, &hardware.driveMotor2, &hardware.driveMotor3, &hardware.driveMotor4};
    // //commands

    commands::HeroDrawCommand draw{drivers, &ui, &gimbal, &flywheel, &indexer, &drivetrain};
    // commands::AutoAimCommand autoCommand{drivers, &gimbal, &jetson};
    // commands::AutoAimAndFireCommand autoFireCommand{drivers, &gimbal, &indexer, &cv};

    commands::JoystickMoveCommand lookJoystick{drivers, &gimbal};
    commands::MouseMoveCommand lookMouse{drivers, &gimbal};
    commands::GimbalStopCommand stopGimbal{drivers, &gimbal};

    commands::ShooterStartCommand shooterStart{drivers, &flywheel};
    commands::ShooterStopCommand shooterStop{drivers, &flywheel};

    commands::IndexerNBallsCommand indexer1Hz{drivers, &indexer, 1, 2};
    commands::IndexerUnjamCommand indexerUnjam{drivers, &indexer};
    commands::IndexerLoadCommand indexerLoadCommand{drivers, &indexer};

    commands::IndexerStopCommand indexerStopCommand{drivers, &indexer};

    //CHANGE NUMBERS LATER
    commands::DrivetrainDriveCommand peekRight{drivers, &drivetrain, &gimbal, commands::DriveMode::PEEK_RIGHT, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand peekLeft{drivers, &drivetrain, &gimbal, commands::DriveMode::PEEK_LEFT, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand drivetrainFollowKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::FOLLOW_TURRET, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand drivetrainFollowJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::FOLLOW_TURRET, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand beybladeJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE2, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand beybladeSlowKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand beybladeFastKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE2, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand noSpinDriveCommand{drivers, &drivetrain, &gimbal, commands::DriveMode::NO_SPIN, commands::ControlMode::CONTROLLER};

    commands::DrivetrainStopCommand stopDriveCommand{drivers, &drivetrain};

    // mappings

    //shooting
    Trigger shootButton{drivers, Remote::Channel::WHEEL, -0.5};
    Trigger unjamButton{drivers, Remote::Channel::WHEEL, 0.5};
    Trigger unjamKey{drivers, Remote::Key::Z}; //or R if based
    Trigger autoAimKey{drivers, MouseButton::RIGHT};
    Trigger shootKey{drivers, MouseButton::LEFT};

    //toggle UI
    Trigger toggleUIKey{drivers, Remote::Key::G};

    //peeking
    Trigger peekLeftButton{drivers, Remote::Key::Q};
    Trigger peekRightButton{drivers, Remote::Key::E};

    //controller driving
    Trigger joystickDrive0{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP};// = (Trigger(drivers, Remote::Key::Q) & Trigger(drivers, Remote::Key::E)) | Trigger(drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
    Trigger joystickDrive1{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID};
    Trigger joystickDrive2{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN};


    //keyboard driving
    Trigger speedModeKey{drivers, Remote::Key::SHIFT};
    Trigger beybladeType0Key{drivers, Remote::Key::X};
    Trigger beybladeType1Key{drivers, Remote::Key::C};
    Trigger beybladeType2Key{drivers, Remote::Key::V};

    Trigger* triggers[15] = {&peekLeftButton, &peekRightButton, &joystickDrive0, &joystickDrive1, &joystickDrive2, &shootButton, &unjamButton, &unjamKey, &shootKey, &autoAimKey, &speedModeKey, &beybladeType0Key, &beybladeType1Key, &beybladeType2Key, &toggleUIKey};//, &indexSpinButton};

};

}  // namespace robots