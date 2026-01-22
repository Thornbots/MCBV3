#include "robots/RobotControl.hpp"

#if defined(INFANTRY)
#include "robots/infantry/InfantryHardware.hpp"
#else
#include "robots/infantry/MechInfantryHardware.hpp"
#endif

#include "subsystems/ui/UISubsystem.hpp"
#include "subsystems/ui/InfantryDrawCommand.hpp"

#include "subsystems/gimbal/JoystickMoveCommand.hpp"
#include "subsystems/gimbal/MouseMoveCommand.hpp"

#include "subsystems/drivetrain/DrivetrainDriveCommand.hpp"
#include "subsystems/drivetrain/DrivetrainStopCommand.hpp"
#include "subsystems/flywheel/ShooterStartCommand.hpp"
#include "subsystems/flywheel/ShooterStopCommand.hpp"
#include "subsystems/gimbal/JoystickMoveCommand.hpp"
#include "subsystems/gimbal/MouseMoveCommand.hpp"
#include "subsystems/gimbal/GimbalStopCommand.hpp"
#include "subsystems/jetson/AutoAimCommand.hpp"
#include "subsystems/jetson/AutoAimAndFireCommand.hpp"
#include "subsystems/indexer/IndexerNBallsCommand.hpp"
#include "subsystems/indexer/IndexerUnjamCommand.hpp"
#include "subsystems/indexer/IndexerStopCommand.hpp"
#include "subsystems/servo/ServoSubsystem.hpp"
#include "subsystems/servo/OpenServoCommand.hpp"
#include "subsystems/servo/CloseServoCommand.hpp"
#include "util/trigger.hpp"


#include "drivers.hpp"
int rawEncoder = 0;
namespace robots {
class InfantryControl : public ControlInterface {
public:
    // pass drivers back to root robotcontrol to store
    InfantryControl(src::Drivers *drivers) : drivers(drivers), hardware(InfantryHardware{drivers}) {}
    // functions we are using
    void initialize() override {
        // Initialize subsystems (registration is internal)
        gimbal.initialize();
        flywheel.initialize();
        indexer.initialize();
        drivetrain.initialize();
        ui.initialize();
        servo.initialize();
        jetson.initialize();

        drivers->commandScheduler.addCommand(&closeServo); //close servo so I stop getting carbon splinters

        // Run startup commands
        gimbal.setDefaultCommand(&stopGimbal);
        flywheel.setDefaultCommand(&shooterStop);
        drivetrain.setDefaultCommand(&stopDriveCommand);
        indexer.setDefaultCommand(&indexerStop);

        shootButton.onTrue(&shooterStart)->whileTrue(&indexer10Hz)->onTrue(&closeServo);
        unjamButton.whileTrue(&indexerUnjam)->onTrue(&openServo);

        stopFlywheelTrigger.onTrue(&shooterStop);

        // Mouse and Keyboard mappings
        unjamKey.whileTrue(&indexerUnjam)->onTrue(&openServo);
        onlyCloseLidKey.onTrue(&closeServo);
        shootRegKey.whileTrue(&indexer10Hz)->onTrue(&shooterStart)->onTrue(&closeServo);
        shootFastKey.whileTrue(&indexer20Hz)->onTrue(&shooterStart)->onTrue(&closeServo);
        autoAimKey.whileTrue(&autoCommand)->onFalse(&lookMouse)->onTrue(&shooterStart)->onTrue(&closeServo);
        // implement speed mode

        toggleUIKey.onTrue(&draw)->onTrue(&drivetrainFollowKeyboard)->onTrue(&lookMouse); //press g to start robot
        // drivers->commandScheduler.addCommand(&draw); //tries to draw immediately, doesn't always work well

        // drive commands and also enable mouse looking

        peekLeftButton.onTrue(&peekLeft);//->onFalse(&beybladeKeyboard);
        peekRightButton.onTrue(&peekRight);//->onFalse(&beybladeKeyboard);
        peekNoneButton.onTrue(&beybladeKeyboard); //makes it so that the driver can be sloppy when they swap peeking directions, they are allowed to press q and e at the same time

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
    InfantryHardware hardware;


    // Subsystems
    subsystems::UISubsystem ui{drivers};
    subsystems::GimbalSubsystem gimbal{drivers, &hardware.yawMotor, &hardware.pitchMotor};
    subsystems::FlywheelSubsystem flywheel{drivers, &hardware.flywheelMotor1, &hardware.flywheelMotor2};
    subsystems::IndexerSubsystem indexer{drivers, &hardware.indexMotor, false};
    subsystems::DrivetrainSubsystem drivetrain{drivers, &hardware.driveMotor1, &hardware.driveMotor2, &hardware.driveMotor3, &hardware.driveMotor4};
    subsystems::ServoSubsystem servo{drivers, &hardware.servo};
    subsystems::JetsonSubsystem jetson{drivers, &gimbal};


    // //commands
    commands::InfantryDrawCommand draw{drivers, &ui, &gimbal, &flywheel, &indexer, &drivetrain, &servo};
    commands::AutoAimCommand autoCommand{drivers, &gimbal, &jetson};
    // commands::AutoAimAndFireCommand autoFireCommand{drivers, &gimbal, &indexer, &cv};

    commands::JoystickMoveCommand lookJoystick{drivers, &gimbal};
    commands::JoystickMoveCommand lookJoystickOffset{drivers, &gimbal, true};
    commands::MouseMoveCommand lookMouse{drivers, &gimbal};
    commands::GimbalStopCommand stopGimbal{drivers, &gimbal};

    commands::ShooterStartCommand shooterStart{drivers, &flywheel};
    commands::ShooterStopCommand shooterStop{drivers, &flywheel};

    commands::IndexerNBallsCommand indexer10Hz{drivers, &indexer, -1, 10};
    commands::IndexerNBallsCommand indexer20Hz{drivers, &indexer, -1, 20};
    commands::IndexerUnjamCommand indexerUnjam{drivers, &indexer};

    commands::IndexerStopCommand indexerStop{drivers, &indexer};

    //CHANGE NUMBERS LATER
    commands::DrivetrainDriveCommand peekRight{drivers, &drivetrain, &gimbal, commands::DriveMode::PEEK_RIGHT, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand peekLeft{drivers, &drivetrain, &gimbal, commands::DriveMode::PEEK_LEFT, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand drivetrainFollowKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::FOLLOW_TURRET, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand drivetrainFollowJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::FOLLOW_TURRET, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand beybladeJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand beybladeKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand noSpinDriveCommand{drivers, &drivetrain, &gimbal, commands::DriveMode::NO_SPIN, commands::ControlMode::CONTROLLER};

    commands::DrivetrainStopCommand stopDriveCommand{drivers, &drivetrain};

    // Servo
    commands::OpenServoCommand openServo{drivers, &servo};
    commands::CloseServoCommand closeServo{drivers, &servo};

    //mappings

    //shooting
    Trigger shootButton{drivers, Remote::Channel::WHEEL, -0.5};
    Trigger unjamButton{drivers, Remote::Channel::WHEEL, 0.5};
    Trigger unjamKey{drivers, Remote::Key::Z}; //or R if based
    Trigger onlyCloseLidKey{drivers, Remote::Key::CTRL}; //blame peter
    Trigger autoAimKey{drivers, MouseButton::RIGHT};
    Trigger shootKey{drivers, MouseButton::LEFT};
    Trigger shootFastKey = shootKey & !onlyCloseLidKey;
    Trigger shootRegKey = shootKey & onlyCloseLidKey;

    Trigger scrollUp{drivers, MouseScrollDirection::UP};
    Trigger scrollDown{drivers, MouseScrollDirection::DOWN};

    //toggle UI
    Trigger toggleUIKey{drivers, Remote::Key::G};

    //peeking
    Trigger peekLeftButton{drivers, Remote::Key::Q};
    Trigger peekRightButton{drivers, Remote::Key::E};
    Trigger peekNoneButton = !(peekLeftButton|peekRightButton);
    
    Trigger joystickLook0{drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP};
    Trigger joystickLook1{drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID};
    Trigger joystickLook2{drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN};

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

    Trigger* triggers[24] = {&joystickLook0, &joystickLook1, &joystickLook2, &peekLeftButton, &peekRightButton, &peekNoneButton, &joystickDrive0, &joystickDrive1, &joystickDrive2, &shootButton, &unjamButton, &onlyCloseLidKey, &unjamKey, &shootKey, &shootRegKey, &shootFastKey, &autoAimKey, &stopBeybladeKey, &beybladeType1Key, &beybladeType2Key, &scrollUp, &scrollDown, &startBeybladeKey, &toggleUIKey};//, &indexSpinButton};

private:
    bool wasControllerModeBeforeRecal;
};

}  // namespace robots