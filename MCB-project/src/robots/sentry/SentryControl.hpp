#pragma once
#include "robots/RobotControl.hpp"
#include "robots/sentry/SentryHardware.hpp"
#include "subsystems/drivetrain/DrivetrainDriveCommand.hpp"
#include "subsystems/drivetrain/DrivetrainStopCommand.hpp"
#include "subsystems/drivetrain/MoveToPositionCommand.hpp"
#include "subsystems/drivetrain/SimpleAutoDriveCommand.hpp"
#include "subsystems/flywheel/ShooterStartCommand.hpp"
#include "subsystems/flywheel/ShooterStopCommand.hpp"
#include "subsystems/gimbal/GimbalStopCommand.hpp"
#include "subsystems/gimbal/JoystickMoveCommand.hpp"
#include "subsystems/gimbal/MouseMoveCommand.hpp"
#include "subsystems/indexer/IndexerIdleCommand.hpp"
#include "subsystems/indexer/IndexerNBallsCommand.hpp"
#include "subsystems/indexer/IndexerStopCommand.hpp"
#include "subsystems/indexer/IndexerUnjamCommand.hpp"
#include "subsystems/indexer/SingleIndexerSubsystem.hpp"
#include "subsystems/jetson/AutoAimAndFireCommand.hpp"
#include "subsystems/jetson/AutoDriveCommand.hpp"
#include "subsystems/jetson/JetsonSubsystem.hpp"
#include "subsystems/jetson/MappingCommand.hpp"
#include "subsystems/odometry/OdometryPointForwardsCommand.hpp"
#include "subsystems/odometry/OdometryStopCommand.hpp"
#include "subsystems/ui/SentryDrawCommand.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/trigger.hpp"

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
        ui.initialize();
        odo.initialize();

        // Run startup commands
        gimbal.setDefaultCommand(&stopGimbal);
        flywheel.setDefaultCommand(&shooterStop);
        drivetrain.setDefaultCommand(&stopDriveCommand);
        indexer.setDefaultCommand(&indexerIdle);
        odo.setDefaultCommand(&odoStop);

        controllerShootTrigger.onTrue(&shooterStart)->onTrue(&indexerSingle);
        controllerUnjamTrigger.whileTrue(&indexerUnjam);
        stopFlywheelTrigger.onTrue(&shooterStop);

        controllerAutoDriveTrigger.onTrue(&simpleAutoDrive)->onTrue(&odoPointForwards);       // right switch up
        controllerDriveNoSpinTrigger.onTrue(&noSpinDriveCommand)->onTrue(&odoPointForwards);  // right switch mid
        controllerDriveBeybladeTrigger.onTrue(&beybladeJoystick)->onTrue(&odoPointForwards);  // right switch down

        controllerAutoFireTrigger.onTrue(&autoFire);                      // left switch up
        controllerLookJoystickTrigger.onTrue(&lookJoystick);              // left switch mid
        controllerLookJoystickOffsetTrigger.onTrue(&lookJoystickOffset);  // left switch down

        isStopped = false;

        // Mouse and Keyboard mappings (Sentry as Standard)
        keyboardUnjamTrigger.whileTrue(&indexerUnjam);
        mouseShootSingleTrigger.whileTrue(&indexerSingle)->onTrue(&shooterStart);
        mouseShootFastTrigger.whileTrue(&indexer20Hz)->onTrue(&shooterStart);
        mouseAimTrigger.onTrue(&shooterStart);
        // implement speed mode

        keyboardUITrigger.onTrue(&draw)->onTrue(&drivetrainFollowKeyboard)->onTrue(&lookMouse);  // press g to start robot
        drivers->commandScheduler.addCommand(&draw);                                             // tries to draw immediately, doesn't always work well

        // drive commands and also enable mouse looking
        keyboardPeekLeftTrigger.onTrue(&peekLeft);
        keyboardPeekRightTrigger.onTrue(&peekRight);
        keyboardPeekNoneTrigger.onTrue(&beybladeKeyboard);  // makes it so that the driver can be sloppy when they swap peeking directions, they are allowed to press q and e at the same time

        keyboardStopBeybladeTrigger.onTrue(&drivetrainFollowKeyboard)->onTrue(&lookMouse);
        startBeybladeKey.onTrue(&beybladeKeyboard)->onTrue(&lookMouse);

        // recal when match is close to starting
        // drivers->recal.requestRecalibration();
    }

    bool rescheduleMoveJoystick = false;
    bool rescheduleMoveKeyboard = false;
    bool rescheduleAutoDrive = false;
    bool rescheduleSimpleAutoDrive = false;

    bool rescheduleAutoFire = false;
    bool rescheduleLookJoystick = false;
    bool rescheduleLookJoystickOffset = false;
    bool rescheduleLookMouse = false;

    void update() override {
        if (drivers->mapping.isActive()) return;
        if (isStopped) return;

        for (Trigger* trigger : triggers) {
            trigger->update();
        }

        rescheduleMoveJoystick = drivetrain.isInControllerMode;  // drivetrainFollowJoystick
        rescheduleMoveKeyboard = drivetrain.isInKeyboardMode;    // drivetrainFollowKeyboard
        rescheduleAutoDrive = drivers->commandScheduler.isCommandScheduled(&autoDrive);
        rescheduleSimpleAutoDrive = drivers->commandScheduler.isCommandScheduled(&simpleAutoDrive);

        rescheduleAutoFire = drivers->commandScheduler.isCommandScheduled(&autoFire);
        rescheduleLookJoystick = drivers->commandScheduler.isCommandScheduled(&lookJoystick);
        rescheduleLookJoystickOffset = drivers->commandScheduler.isCommandScheduled(&lookJoystickOffset);
        rescheduleLookMouse = drivers->commandScheduler.isCommandScheduled(&lookMouse);

        // if we don't have ref uart or we aren't in a match or we aren't currently in game, we are able to stop flywheels by buttons
        if (!drivers->refSerial.getRefSerialReceivingData() || drivers->refSerial.getGameData().gameType != RefSerialData::Rx::GameType::ROBOMASTER_RMUL_3V3 ||
            drivers->refSerial.getGameData().gameStage != RefSerialData::Rx::GameStage::IN_GAME) {
            stopFlywheelTrigger.update();
        }
    }

    void updateMapping() override {
        const auto now = tap::arch::clock::getTimeMilliseconds();
        const bool fresh = drivers->i2c.odom.isFresh(now, src::POD_STALE_MS);
        if (!fresh) drivers->mapping.reportOdomFault();
        if (drivers->mapping.getState() == src::MappingMachine::State::OdomFault) {
            drivers->buzzer.setReady(false);
            drivers->leds.set(tap::gpio::Leds::Red, true);
            drivers->leds.set(tap::gpio::Leds::Green, false);
            drivers->leds.set(tap::gpio::Leds::Blue, false);
        }
        drivers->buzzer.update(now, fresh);
        // main skips the scheduler (and Jetson refresh) until the IMU is ready.
        if (!drivers->recal.getIsImuReady()) jetson.serviceMappingPing();
        if (!drivers->mapping.isActive() || isStopped || !drivers->remote.isConnected()) return;
        if (!drivers->commandScheduler.isCommandScheduled(&mappingCommand)) drivers->commandScheduler.addCommand(&mappingCommand);
    }

    void stopForImuRecal() override {
        drivers->commandScheduler.addCommand(&stopGimbal);
        drivers->commandScheduler.addCommand(&shooterStop);
        drivers->commandScheduler.addCommand(&stopDriveCommand);
        drivers->commandScheduler.addCommand(&indexerStop);
        drivers->commandScheduler.addCommand(&odoStop);
        isStopped = true;
    }

    void resumeAfterImuRecal() override {
        isStopped = false;
        gimbal.clearBuildup();
        gimbal.reZeroYaw();
        // Need to find a better way to do this, like saving a reference
        // to the currently running command, and rescheduling it here.
        // But I don't know how to get the currently running command on a subsystem.
        if (rescheduleMoveJoystick) drivers->commandScheduler.addCommand(&drivetrainFollowJoystick);
        if (rescheduleMoveKeyboard) drivers->commandScheduler.addCommand(&drivetrainFollowKeyboard);
        if (rescheduleAutoDrive) drivers->commandScheduler.addCommand(&autoDrive);
        if (rescheduleSimpleAutoDrive) drivers->commandScheduler.addCommand(&simpleAutoDrive);

        if (rescheduleAutoFire)
            drivers->commandScheduler.addCommand(&autoFire);
        else
            drivers->commandScheduler.addCommand(&indexerIdle);
        if (rescheduleLookJoystick) drivers->commandScheduler.addCommand(&lookJoystick);
        if (rescheduleLookJoystickOffset) drivers->commandScheduler.addCommand(&lookJoystickOffset);
        if (rescheduleLookMouse) drivers->commandScheduler.addCommand(&lookMouse);
        drivers->commandScheduler.addCommand(&odoPointForwards);
        update();
    }

    bool isStopped = true;

    src::Drivers* drivers;
    SentryHardware hardware;

    // subsystems
    subsystems::UISubsystem ui{drivers};
    subsystems::GimbalSubsystem gimbal{drivers, &hardware.yawMotor, &hardware.pitchMotor};
    subsystems::FlywheelSubsystem flywheel{drivers, &hardware.flywheelMotor1, &hardware.flywheelMotor2};
    subsystems::SingleIndexerSubsystem indexer{drivers, &hardware.indexMotor, true};  // sentry homes
    subsystems::DrivetrainSubsystem drivetrain{drivers, &hardware.driveMotor1, &hardware.driveMotor2, &hardware.driveMotor3, &hardware.driveMotor4};
    subsystems::OdometrySubsystem odo{drivers, &hardware.odoMotor};
    subsystems::JetsonSubsystem jetson{drivers, &gimbal, &odo};

    // commands
    commands::MappingCommand mappingCommand{drivers, &drivetrain, &gimbal, &flywheel, &indexer, &odo};

    commands::JoystickMoveCommand lookJoystick{drivers, &gimbal};
    commands::JoystickMoveCommand lookJoystickOffset{drivers, &gimbal, true};
    // commands::MouseMoveCommand lookMouse{drivers, &gimbal};
    commands::AutoAimAndFireCommand lookMouse{drivers, &gimbal, &indexer, &flywheel, &jetson, &odo, &autoDrive, true};
    commands::GimbalStopCommand stopGimbal{drivers, &gimbal};
    commands::AutoDriveCommand autoDrive{drivers, &drivetrain, &gimbal, &jetson, &odo};
    commands::SimpleAutoDriveCommand simpleAutoDrive{drivers, &drivetrain, &gimbal, &odo, commands::SimpleAutoDriveCommand::TargetMode::SPOOKY_SCRIM_2V2};
    commands::AutoAimAndFireCommand autoFire{drivers, &gimbal, &indexer, &flywheel, &jetson, &odo, &autoDrive, false};

    commands::SentryDrawCommand draw{drivers, &ui, &gimbal, &flywheel, &indexer, &drivetrain, &lookMouse};

    commands::ShooterStartCommand shooterStart{drivers, &flywheel};
    commands::ShooterStopCommand shooterStop{drivers, &flywheel};

    commands::IndexerNBallsCommand indexerSingle{drivers, &indexer, 1, 20};
    commands::IndexerNBallsCommand indexer10Hz{drivers, &indexer, -1, 10};
    commands::IndexerNBallsCommand indexer20Hz{drivers, &indexer, -1, 20};
    commands::IndexerUnjamCommand indexerUnjam{drivers, &indexer};

    commands::IndexerStopCommand indexerStop{drivers, &indexer};
    commands::IndexerIdleCommand indexerIdle{drivers, &indexer};

    commands::OdometryPointForwardsCommand odoPointForwards{drivers, &odo, &gimbal};
    commands::OdometryStopCommand odoStop{drivers, &odo};

    commands::DrivetrainDriveCommand peekRight{drivers, &drivetrain, &gimbal, commands::DriveMode::PEEK_RIGHT, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand peekLeft{drivers, &drivetrain, &gimbal, commands::DriveMode::PEEK_LEFT, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand drivetrainFollowKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::FOLLOW_TURRET, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand drivetrainFollowJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::FOLLOW_TURRET, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand beybladeJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand beybladeKeyboard{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE, commands::ControlMode::KEYBOARD};
    commands::DrivetrainDriveCommand noSpinDriveCommand{drivers, &drivetrain, &gimbal, commands::DriveMode::NO_SPIN, commands::ControlMode::CONTROLLER};

    commands::DrivetrainStopCommand stopDriveCommand{drivers, &drivetrain};

    // mappings

    // shooting
    Trigger controllerShootTrigger{drivers, Remote::Channel::WHEEL, -0.5};
    Trigger controllerUnjamTrigger{drivers, Remote::Channel::WHEEL, 0.5};
    Trigger keyboardUnjamTrigger{drivers, Remote::Key::Z};
    Trigger keyboardModifierTrigger{drivers, Remote::Key::CTRL};
    Trigger mouseAimTrigger{drivers, MouseButton::RIGHT};
    Trigger mouseShootTrigger{drivers, MouseButton::LEFT};
    Trigger mouseShootFastTrigger = mouseShootTrigger & keyboardModifierTrigger;
    Trigger mouseShootSingleTrigger = mouseShootTrigger & !keyboardModifierTrigger;

    Trigger scrollUpTrigger{drivers, MouseScrollDirection::UP};
    Trigger scrollDownTrigger{drivers, MouseScrollDirection::DOWN};

    // toggle UI
    Trigger keyboardUITrigger{drivers, Remote::Key::G};

    // peeking
    Trigger keyboardPeekLeftTrigger{drivers, Remote::Key::Q};
    Trigger keyboardPeekRightTrigger{drivers, Remote::Key::E};
    Trigger keyboardPeekNoneTrigger = !(keyboardPeekLeftTrigger | keyboardPeekRightTrigger);

    // controller driving
    Trigger controllerAutoDriveTrigger{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP};
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

    Trigger* triggers[24] = {
        &controllerLookJoystickTrigger,
        &controllerLookJoystickOffsetTrigger,
        &controllerDriveNoSpinTrigger,
        &controllerDriveBeybladeTrigger,
        &controllerShootTrigger,
        &controllerUnjamTrigger,
        &keyboardUnjamTrigger,
        &controllerAutoFireTrigger,
        &keyboardStopBeybladeTrigger,
        &keyboardBeybladeSpinPriorityTrigger,
        &keyboardBeybladeMovePriorityTrigger,
        &startBeybladeKey,
        &keyboardPeekLeftTrigger,
        &keyboardPeekRightTrigger,
        &keyboardPeekNoneTrigger,
        &scrollUpTrigger,
        &scrollDownTrigger,
        &mouseShootFastTrigger,
        &mouseShootSingleTrigger,
        &mouseShootTrigger,
        &mouseAimTrigger,
        &keyboardUITrigger,
        &keyboardModifierTrigger,
        &controllerAutoDriveTrigger};  //, &indexSpinButton};
};
}  // namespace robots
