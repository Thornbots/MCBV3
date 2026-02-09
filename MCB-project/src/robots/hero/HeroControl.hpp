#include "robots/RobotControl.hpp"

#include "robots/hero/HeroHardware.hpp"

#include "subsystems/ui/UISubsystem.hpp"
#include "subsystems/ui/HeroDrawCommand.hpp"

#include "subsystems/indexer/HeroIndexerSubsystem.hpp"

#include "subsystems/gimbal/JoystickMoveCommand.hpp"
#include "subsystems/gimbal/MouseMoveCommand.hpp"
#include "subsystems/jetson/AutoAimCommand.hpp"

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

#define TRIGGER(name, function) Trigger name{drivers, [this]() {return drivers->inputWrapper.function();}}

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
        jetson.initialize();
        
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
        autoAimKey.whileTrue(&autoCommand)->onFalse(&lookMouse)->onTrue(&shooterStart);
        // implement speed mode

        toggleUIKey.onTrue(&draw)->onTrue(&drivetrainFollowKeyboard)->onTrue(&lookMouse); //press g to start robot
        keyboardTakeControl.onTrue(&drivetrainFollowKeyboard)->onTrue(&lookMouse);
        // drivers->commandScheduler.addCommand(&draw);
   
        // drive commands and also enable mouse looking

        peekLeftButton.onTrue(&peekLeft)->onFalse(&beybladeKeyboard);
        peekRightButton.onTrue(&peekRight)->onFalse(&beybladeKeyboard);

        stopBeybladeKey.onTrue(&drivetrainFollowKeyboard)->onTrue(&lookMouse);
        startBeybladeKey.onTrue(&beybladeKeyboard)->onTrue(&lookMouse);
 
        joystickDrive0.onTrue(&noSpinDriveCommand);
        joystickDrive1.onTrue(&drivetrainFollowJoystick);
        joystickDrive2.onTrue(&beybladeJoystick);

        joystickLook0.onTrue(&lookJoystick); //looks horizontal
        joystickLook1.onTrue(&lookJoystick); //looks horizontal
        joystickLook2.onTrue(&lookJoystickOffset); //looks downward to fit in sizing box

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
        wasControllerModeBeforeRecal = drivetrain.isInControllerMode;
        drivers->commandScheduler.addCommand(&stopGimbal);
        drivers->commandScheduler.addCommand(&shooterStop);
        drivers->commandScheduler.addCommand(&stopDriveCommand);
        drivers->commandScheduler.addCommand(&indexerStop);
        isStopped = true;
    }

    void resumeAfterImuRecal() override {
        isStopped = false;
        gimbal.clearBuildup();
        gimbal.reZeroYaw();
        if (wasControllerModeBeforeRecal) {
            drivers->commandScheduler.addCommand(&lookJoystickOffset);
        } else {
            drivers->commandScheduler.addCommand(&lookMouse);
        }
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
    subsystems::JetsonSubsystem jetson{drivers, &gimbal};
    // //commands

    commands::HeroDrawCommand draw{drivers, &ui, &gimbal, &flywheel, &indexer, &drivetrain};
    commands::AutoAimCommand autoCommand{drivers, &gimbal, &jetson};
    // commands::AutoAimAndFireCommand autoFireCommand{drivers, &gimbal, &indexer, &cv};

    commands::JoystickMoveCommand lookJoystick{drivers, &gimbal};
    commands::JoystickMoveCommand lookJoystickOffset{drivers, &gimbal, true};
    commands::MouseMoveCommand lookMouse{drivers, &gimbal};
    commands::GimbalStopCommand stopGimbal{drivers, &gimbal};

    commands::ShooterStartCommand shooterStart{drivers, &flywheel};
    commands::ShooterStopCommand shooterStop{drivers, &flywheel};

    commands::IndexerNBallsCommand indexerSemi{drivers, &indexer, 1, 20}; //semiauto, each click is one shot
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
    TRIGGER(shootButton, isShootPressed);
    TRIGGER(unjamButton, isUnjamPressed);
    TRIGGER(unjamKey, isUnjamKeyPressed);

    TRIGGER(autoAimKey, isAutoAimKeyPressed);
    TRIGGER(shootKey, isShootKeyPressed);

    TRIGGER(scrollUp, isScrollUp);
    TRIGGER(scrollDown, isScrollDown);

    //toggle UI
    TRIGGER(toggleUIKey, isToggleUIPressed);

    //peeking
    TRIGGER(peekLeftButton, isPeekLeftPressed);
    TRIGGER(peekRightButton, isPeekRightPressed);

    //controller driving
    TRIGGER(joystickDrive0, isRightSwitchUp);
    TRIGGER(joystickDrive1, isRightSwitchMid);
    TRIGGER(joystickDrive2, isRightSwitchDown);

    TRIGGER(joystickLook0, isLeftSwitchUp);
    TRIGGER(joystickLook1, isLeftSwitchMid);
    TRIGGER(joystickLook2, isLeftSwitchDown);

    Trigger keyboardTakeControl = Trigger{drivers, [this]() {
        return (drivers->inputWrapper.isAnyKeyPressed() || drivers->remote.getMouseX() != 0 || drivers->remote.getMouseY() != 0) && drivetrain.isInControllerMode;
    }};

    //keyboard driving
    // Trigger speedModeKey{drivers, Remote::Key::SHIFT}; //drivetrain drive command reads shift
    TRIGGER(stopBeybladeKey, isStopBeybladePressed);
    TRIGGER(beybladeType1Key, isBeybladeSpinPressed); //most beyblade, checked in DrivetrainDriveCommand
    TRIGGER(beybladeType2Key, isBeybladeMovePressed); //most translation, checked in DrivetrainDriveCommand
    Trigger startBeybladeKey = beybladeType1Key | beybladeType2Key | scrollUp | scrollDown;

    Trigger stopFlywheelTrigger = unjamButton | unjamKey; //doesn't get added to the list of triggers, is special, during a match the only way to turn off flywheels is to turn off the remote

    Trigger* triggers[21] = {&keyboardTakeControl, &peekLeftButton, &peekRightButton, &joystickDrive0, &joystickDrive1, &joystickDrive2, &joystickLook0, &joystickLook1, &joystickLook2, &shootButton, &unjamButton, &unjamKey, &shootKey, &autoAimKey, &stopBeybladeKey, &beybladeType1Key, &beybladeType2Key, &scrollUp, &scrollDown, &startBeybladeKey, &toggleUIKey};//, &indexSpinButton};
private:
    bool wasControllerModeBeforeRecal;
};

}  // namespace robots