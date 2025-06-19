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
        indexer.initialize();
        drivetrain.initialize();
        ui.initialize();
        
        // Run startup commands
        gimbal.setDefaultCommand(&stopGimbal);
        flywheel.setDefaultCommand(&shooterStop);
        drivetrain.setDefaultCommand(&stopDriveCommand);
        indexer.setDefaultCommand(&indexerLoad);
        

        // Mouse and Keyboard mappings
        unjamKey.whileTrue(&indexerUnjam)->onFalse(&indexerLoad);
        shootKey.onTrue(&indexerSemi)->onTrue(&shooterStart)->onFalse(&indexerLoad);
        unjamButton.whileTrue(&indexerUnjam)->onFalse(&indexerLoad);
        shootButton.whileTrue(&indexerAuto)->onTrue(&shooterStart)->onFalse(&indexerLoad);
        stopFlywheelTrigger.onTrue(&shooterStop);
        //autoAimKey./*whileTrue(&autoCommand)->*/onFalse(&lookMouse)->whileTrue(&shooterStart);
        // implement speed mode

        toggleUIKey.onTrue(&draw)->onTrue(&drivetrainFollowKeyboard)->onTrue(&lookMouse); //press g to start robot
        // drivers->commandScheduler.addCommand(&draw);
   
        // drive commands and also enable mouse looking

        peekLeftButton.onTrue(&peekLeft)->onFalse(&beybladeKeyboard);
        peekRightButton.onTrue(&peekRight)->onFalse(&beybladeKeyboard);

        stopBeybladeKey.onTrue(&drivetrainFollowKeyboard)->onTrue(&lookMouse);
        startBeybladeKey.onTrue(&beybladeKeyboard)->onTrue(&lookMouse);
 
        joystickDrive0.onTrue(&noSpinDriveCommand)->onTrue(&lookJoystick);
        joystickDrive1.onTrue(&drivetrainFollowJoystick)->onTrue(&lookJoystick);
        joystickDrive2.onTrue(&beybladeJoystick)->onTrue(&lookJoystick);

        isStopped = false;
    }

    void update() override {
        if(isStopped)
            return;

        for (Trigger* trigger : triggers) {
            trigger->update();
        }
        
        //if we don't have ref uart or we do and we aren't currently in game, we are able to stop flywheels by buttons
        if(!drivers->refSerial.getRefSerialReceivingData() || drivers->refSerial.getGameData().gameStage!=RefSerialData::Rx::GameStage::IN_GAME){
            stopFlywheelTrigger.update();
        }
    }

    void stopForImuRecal() override {
        drivers->commandScheduler.addCommand(&stopGimbal);
        drivers->commandScheduler.addCommand(&shooterStop);
        drivers->commandScheduler.addCommand(&stopDriveCommand);
        drivers->commandScheduler.addCommand(&indexerStop);
        isStopped = true;
    }

    void resumeAfterImuRecal() override {
        isStopped = false;
        gimbal.clearBuildup();
        drivers->commandScheduler.addCommand(&lookMouse);
        drivers->commandScheduler.addCommand(&drivetrainFollowKeyboard);
        update();
    }

    bool isStopped = true;

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

    commands::IndexerNBallsCommand indexerSemi{drivers, &indexer, 1, 2}; //semiauto, each click is one shot
    commands::IndexerNBallsCommand indexerAuto{drivers, &indexer, -1, 2};//full auto, holding the wheel it the forward position shoots as long as it is held
    commands::IndexerUnjamCommand indexerUnjam{drivers, &indexer};
    commands::IndexerLoadCommand indexerLoad{drivers, &indexer};

    commands::IndexerStopCommand indexerStop{drivers, &indexer}; //stop is unused

    //CHANGE NUMBERS LATER
    commands::DrivetrainDriveCommand peekRight{drivers, &drivetrain, &gimbal, commands::DriveMode::PEEK_RIGHT, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand peekLeft{drivers, &drivetrain, &gimbal, commands::DriveMode::PEEK_LEFT, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand drivetrainFollowKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::FOLLOW_TURRET, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand drivetrainFollowJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::FOLLOW_TURRET, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand beybladeJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand beybladeKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand noSpinDriveCommand{drivers, &drivetrain, &gimbal, commands::DriveMode::NO_SPIN, commands::ControlMode::CONTROLLER};

    commands::DrivetrainStopCommand stopDriveCommand{drivers, &drivetrain};

    // mappings

    //shooting
    Trigger shootButton{drivers, Remote::Channel::WHEEL, -0.5};
    Trigger unjamButton{drivers, Remote::Channel::WHEEL, 0.5};
    Trigger unjamKey{drivers, Remote::Key::Z}; //or R if based
    Trigger autoAimKey{drivers, MouseButton::RIGHT};
    Trigger shootKey{drivers, MouseButton::LEFT};

    Trigger scrollUp{drivers, MouseScrollDirection::UP};
    Trigger scrollDown{drivers, MouseScrollDirection::DOWN};

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
    // Trigger speedModeKey{drivers, Remote::Key::SHIFT}; //drivetrain drive command reads shift
    Trigger stopBeybladeKey{drivers, Remote::Key::X};
    Trigger beybladeType1Key{drivers, Remote::Key::C}; //most beyblade, checked in DrivetrainDriveCommand
    Trigger beybladeType2Key{drivers, Remote::Key::V}; //most translation, checked in DrivetrainDriveCommand
    Trigger startBeybladeKey = beybladeType1Key | beybladeType2Key | scrollUp | scrollDown;

    Trigger stopFlywheelTrigger = unjamButton | unjamKey; //doesn't get added to the list of triggers, is special, during a match the only way to turn off flywheels is to turn off the remote

    Trigger* triggers[17] = {&peekLeftButton, &peekRightButton, &joystickDrive0, &joystickDrive1, &joystickDrive2, &shootButton, &unjamButton, &unjamKey, &shootKey, &autoAimKey, &stopBeybladeKey, &beybladeType1Key, &beybladeType2Key, &scrollUp, &scrollDown, &startBeybladeKey, &toggleUIKey};//, &indexSpinButton};

};

}  // namespace robots