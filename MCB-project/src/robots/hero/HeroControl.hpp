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
        //gimbal.initialize();
        flywheel.initialize();
        indexer.initialize();
        //drivetrain.initialize();
        ui.initialize();

        // Run startup commands
        gimbal.setDefaultCommand(&look);
        flywheel.setDefaultCommand(&shooterStop);
        indexer.setDefaultCommand(&indexerStopCommand);
        

        shootButton.whileTrue(&indexer10Hz)->onTrue(&shooterStart)->onFalse(&indexerLoadCommand);
        unjamButton.whileTrue(&indexerUnjam)->onTrue(&shooterStop)->onFalse(&indexerLoadCommand);


        // //peeking
        peekLeftButton.whileTrue(&peekLeft);
        peekRightButton.whileTrue(&peekRight);
        
        // drivers->commandMapper.addMap(&controllerToKeyboardMouseMapping);
        // drivers->commandScheduler.addCommand(&indexerUnjam);
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

    commands::JoystickMoveCommand look{drivers, &gimbal};
    commands::MouseMoveCommand look2{drivers, &gimbal};

    commands::ShooterStartCommand shooterStart{drivers, &flywheel};
    commands::ShooterStopCommand shooterStop{drivers, &flywheel};

    commands::IndexerNBallsCommand indexer10Hz{drivers, &indexer, -1, 1};
    commands::IndexerUnjamCommand indexerUnjam{drivers, &indexer};
    commands::IndexerStopCommand indexerStopCommand{drivers, &indexer};
    commands::IndexerLoadCommand indexerLoadCommand{drivers, &indexer};

    //CHANGE NUMBERS LATER
    commands::DrivetrainDriveCommand peekRight{drivers, &drivetrain, &gimbal, commands::DriveMode::PEEK_RIGHT, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand peekLeft{drivers, &drivetrain, &gimbal, commands::DriveMode::PEEK_LEFT, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand peekCenterWASD{drivers, &drivetrain, &gimbal, commands::DriveMode::FOLLOW_TURRET, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand peekCenterJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::FOLLOW_TURRET, commands::ControlMode::CONTROLLER};

    commands::DrivetrainStopCommand stopDriveCommand{drivers, &drivetrain};

    // mappings


    //maybe change later

    //peeking
    Trigger peekLeftButton{drivers, Remote::Key::Q};
    Trigger peekRightButton{drivers, Remote::Key::E};
    Trigger peekCenterButton{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP};// = (Trigger(drivers, Remote::Key::Q) & Trigger(drivers, Remote::Key::E)) | Trigger(drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);

    //shooting
    //shooting
    Trigger shootButton{drivers, Remote::Channel::WHEEL, 0.5};// = Trigger(drivers, Remote::Channel::WHEEL, 0.5) | Trigger(drivers, MouseButton::LEFT);
    //unjamming we know the button reacts 
    Trigger unjamButton{drivers, Remote::Channel::WHEEL, -0.5};// = Trigger(drivers, Remote::Channel::WHEEL, -0.5) | Trigger(drivers, MouseButton::RIGHT);

    //Trigger indexSpinButton;// = Trigger(drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP) & Trigger(drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
    Trigger* triggers[5] = {&peekLeftButton, &peekRightButton, &peekCenterButton, &shootButton, &unjamButton};//, &indexSpinButton};


    // ToggleCommandMapping controllerToKeyboardMouseMapping{drivers, {&look2}, RemoteMapState({Remote::Key::CTRL, Remote::Key::Z})};

};

}  // namespace robots
