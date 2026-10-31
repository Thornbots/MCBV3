#pragma once
#include "robots/RobotControl.hpp"
#include "robots/hero/HeroHardware.hpp"
#include "subsystems/drivetrain/DrivetrainDriveCommand.hpp"
#include "subsystems/drivetrain/DrivetrainStopCommand.hpp"
#include "subsystems/flywheel/ShooterStartCommand.hpp"
#include "subsystems/flywheel/ShooterStopCommand.hpp"
#include "subsystems/gimbal/GimbalStopCommand.hpp"
#include "subsystems/gimbal/JoystickMoveCommand.hpp"
#include "subsystems/gimbal/MouseMoveCommand.hpp"
#include "subsystems/indexer/HeroIndexerSubsystem.hpp"
#include "subsystems/indexer/IndexerIdleCommand.hpp"
#include "subsystems/indexer/IndexerNBallsCommand.hpp"
#include "subsystems/indexer/IndexerStopCommand.hpp"
#include "subsystems/indexer/IndexerUnjamCommand.hpp"
#include "subsystems/jetson/AutoAimAndFireCommand.hpp"
#include "subsystems/ui/HeroDrawCommand.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/trigger.hpp"

#include "drivers.hpp"

namespace robots {
class HeroControl : public ControlInterface {
public:
    // pass drivers back to root robotcontrol to store
    HeroControl(src::Drivers* drivers) : drivers(drivers), hardware(HeroHardware{drivers}) {}
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
        indexer.setDefaultCommand(&indexerIdle);

        // Mouse and Keyboard mappings
        keyboardUnjamTrigger.whileTrue(&indexerUnjam);
        mouseShootTrigger.onTrue(&indexerSemi)->onTrue(&shooterStart);
        controllerUnjamTrigger.whileTrue(&indexerUnjam);
        controllerShootTrigger.onTrue(&indexerSemi)->onTrue(&shooterStart);
        stopFlywheelTrigger.onTrue(&shooterStop);
        mouseAimTrigger.onTrue(&shooterStart);

        keyboardUITrigger.onTrue(&draw)->onTrue(&drivetrainFollowKeyboard)->onTrue(&lookMouse);  // press g to start robot
        drivers->commandScheduler.addCommand(&draw);

        // drive commands and also enable mouse looking
        keyboardPeekLeftTrigger.onTrue(&peekLeft)->onFalse(&beybladeKeyboard);
        keyboardPeekRightTrigger.onTrue(&peekRight)->onFalse(&beybladeKeyboard);

        keyboardStopBeybladeTrigger.onTrue(&drivetrainFollowKeyboard)->onTrue(&lookMouse);
        startBeybladeKey.onTrue(&beybladeKeyboard)->onTrue(&lookMouse);

        controllerDrivePointForwardTrigger.onTrue(&drivetrainFollowJoystick);  // right switch up
        controllerDriveNoSpinTrigger.onTrue(&noSpinDriveCommand);              // right switch mid
        controllerDriveBeybladeTrigger.onTrue(&beybladeJoystick);              // right switch down

        controllerAutoFireTrigger.onTrue(&autoFire);                      // left switch up
        controllerLookJoystickTrigger.onTrue(&lookJoystick);              // left switch mid
        controllerLookJoystickOffsetTrigger.onTrue(&lookJoystickOffset);  // left switch down

        isStopped = false;
    }

    bool rescheduleMoveJoystick = false;
    bool rescheduleMoveKeyboard = false;

    bool rescheduleLookJoystick = false;
    bool rescheduleLookJoystickOffset = false;
    bool rescheduleLookMouse = false;

    void update() override {
        if (isStopped) return;

        for (Trigger* trigger : triggers) {
            trigger->update();
        }

        rescheduleMoveJoystick = drivetrain.isInControllerMode;  // drivetrainFollowJoystick
        rescheduleMoveKeyboard = drivetrain.isInKeyboardMode;    // drivetrainFollowKeyboard

        rescheduleLookJoystick = drivers->commandScheduler.isCommandScheduled(&lookJoystick);
        rescheduleLookJoystickOffset = drivers->commandScheduler.isCommandScheduled(&lookJoystickOffset);
        rescheduleLookMouse = drivers->commandScheduler.isCommandScheduled(&lookMouse);

        // if we don't have ref uart or we do and we aren't currently in game, we are able to stop flywheels by buttons
        if (!drivers->refSerial.getRefSerialReceivingData() || drivers->refSerial.getGameData().gameStage != RefSerialData::Rx::GameStage::IN_GAME) {
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
        gimbal.reZeroYaw();
        if (rescheduleMoveJoystick) drivers->commandScheduler.addCommand(&drivetrainFollowJoystick);
        if (rescheduleMoveKeyboard) drivers->commandScheduler.addCommand(&drivetrainFollowKeyboard);

        if (rescheduleLookJoystick) drivers->commandScheduler.addCommand(&lookJoystick);
        if (rescheduleLookJoystickOffset) drivers->commandScheduler.addCommand(&lookJoystickOffset);
        if (rescheduleLookMouse) drivers->commandScheduler.addCommand(&lookMouse);
        drivers->commandScheduler.addCommand(&drivetrainFollowKeyboard);
        drivers->commandScheduler.addCommand(&indexerIdle);
        update();
    }

    bool isStopped = true;

    src::Drivers* drivers;
    HeroHardware hardware;

    // Subsystems
    subsystems::UISubsystem ui{drivers};
    subsystems::GimbalSubsystem gimbal{drivers, &hardware.yawMotor, &hardware.pitchMotor};
    subsystems::FlywheelSubsystem flywheel{drivers, &hardware.flywheelMotor1, &hardware.flywheelMotor2};
    subsystems::HeroIndexerSubsystem indexer{drivers, &hardware.indexTopMotor, &hardware.indexBottomMotor};
    subsystems::DrivetrainSubsystem drivetrain{drivers, &hardware.driveMotor1, &hardware.driveMotor2, &hardware.driveMotor3, &hardware.driveMotor4};
    subsystems::JetsonSubsystem jetson{drivers, &gimbal, nullptr};

    // commands
    commands::HeroDrawCommand draw{drivers, &ui, &gimbal, &flywheel, &indexer, &drivetrain};
    commands::AutoAimAndFireCommand autoFire{drivers, &gimbal, &indexer, &flywheel, &jetson, nullptr, nullptr, false};

    commands::JoystickMoveCommand lookJoystick{drivers, &gimbal};
    commands::JoystickMoveCommand lookJoystickOffset{drivers, &gimbal, true};
    // commands::MouseMoveCommand lookMouse{drivers, &gimbal};
    commands::AutoAimAndFireCommand lookMouse{drivers, &gimbal, &indexer, &flywheel, &jetson, nullptr, nullptr, true};
    commands::GimbalStopCommand stopGimbal{drivers, &gimbal};

    commands::ShooterStartCommand shooterStart{drivers, &flywheel};
    commands::ShooterStopCommand shooterStop{drivers, &flywheel};

    commands::IndexerNBallsCommand indexerSemi{drivers, &indexer, 1, 20};  // semiauto, each click is one shot [Hero ignores the 20, need to fix this]
    commands::IndexerNBallsCommand indexerAuto{drivers, &indexer, -1, 2};  // full auto, holding the wheel it the forward position shoots as long as it is held
    commands::IndexerUnjamCommand indexerUnjam{drivers, &indexer};

    commands::IndexerStopCommand indexerStop{drivers, &indexer};
    commands::IndexerIdleCommand indexerIdle{drivers, &indexer};

    // CHANGE NUMBERS LATER
    commands::DrivetrainDriveCommand peekRight{drivers, &drivetrain, &gimbal, commands::DriveMode::PEEK_RIGHT, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand peekLeft{drivers, &drivetrain, &gimbal, commands::DriveMode::PEEK_LEFT, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand drivetrainFollowKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::NO_SPIN, commands::ControlMode::KEYBOARD}; //temp standard and hero don't have yaw encoder
    commands::DrivetrainDriveCommand drivetrainFollowJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::NO_SPIN, commands::ControlMode::CONTROLLER}; //temp standard and hero don't have yaw encoder
    commands::DrivetrainDriveCommand beybladeJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand beybladeKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand noSpinDriveCommand{drivers, &drivetrain, &gimbal, commands::DriveMode::NO_SPIN, commands::ControlMode::CONTROLLER};

    commands::DrivetrainStopCommand stopDriveCommand{drivers, &drivetrain};

    // mappings

    // shooting
    Trigger controllerShootTrigger{drivers, Remote::Channel::WHEEL, -0.5};
    Trigger controllerUnjamTrigger{drivers, Remote::Channel::WHEEL, 0.5};
    Trigger keyboardUnjamTrigger{drivers, Remote::Key::Z};  // or R if based
    Trigger mouseAimTrigger{drivers, MouseButton::RIGHT};
    Trigger mouseShootTrigger{drivers, MouseButton::LEFT};

    Trigger scrollUpTrigger{drivers, MouseScrollDirection::UP};
    Trigger scrollDownTrigger{drivers, MouseScrollDirection::DOWN};

    // toggle UI
    Trigger keyboardUITrigger{drivers, Remote::Key::G};

    // peeking
    Trigger keyboardPeekLeftTrigger{drivers, Remote::Key::Q};
    Trigger keyboardPeekRightTrigger{drivers, Remote::Key::E};

    // controller driving
    Trigger controllerDrivePointForwardTrigger{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP};
    Trigger controllerDriveNoSpinTrigger{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID};
    Trigger controllerDriveBeybladeTrigger{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN};

    Trigger controllerAutoFireTrigger{drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP};
    Trigger controllerLookJoystickTrigger{drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID};
    Trigger controllerLookJoystickOffsetTrigger{drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN};

    // keyboard driving
    Trigger keyboardStopBeybladeTrigger{drivers, Remote::Key::X};
    Trigger keyboardBeybladeSpinPriorityTrigger{drivers, Remote::Key::C};  // most beyblade, checked in DrivetrainDriveCommand
    Trigger keyboardBeybladeMovePriorityTrigger{drivers, Remote::Key::V};  // most translation, checked in DrivetrainDriveCommand
    Trigger startBeybladeKey = keyboardBeybladeSpinPriorityTrigger | keyboardBeybladeMovePriorityTrigger | scrollUpTrigger | scrollDownTrigger;

    Trigger stopFlywheelTrigger =
        controllerUnjamTrigger | keyboardUnjamTrigger;  // doesn't get added to the list of triggers, is special, during a match the only way to turn off flywheels is to turn off the remote

    Trigger* triggers[20] = {
        &keyboardPeekLeftTrigger,
        &keyboardPeekRightTrigger,
        &controllerDrivePointForwardTrigger,
        &controllerDriveNoSpinTrigger,
        &controllerDriveBeybladeTrigger,
        &controllerAutoFireTrigger,
        &controllerLookJoystickTrigger,
        &controllerLookJoystickOffsetTrigger,
        &controllerShootTrigger,
        &controllerUnjamTrigger,
        &keyboardUnjamTrigger,
        &mouseShootTrigger,
        &mouseAimTrigger,
        &keyboardStopBeybladeTrigger,
        &keyboardBeybladeSpinPriorityTrigger,
        &keyboardBeybladeMovePriorityTrigger,
        &scrollUpTrigger,
        &scrollDownTrigger,
        &startBeybladeKey,
        &keyboardUITrigger};
};

}  // namespace robots