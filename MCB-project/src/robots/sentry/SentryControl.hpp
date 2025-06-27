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
#include "subsystems/drivetrain/MoveToPositionCommand.hpp"
#include "tap/control/sequential_command.hpp"

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
        indexer.setDefaultCommand(&indexerStop);
        odo.setDefaultCommand(&odoStop);

        
        shootButton.onTrue(&shooterStart)->whileTrue(&indexerStart);
        unjamButton.whileTrue(&indexerUnjam);
        stopFlywheelTrigger.onTrue(&shooterStop);

        autoFireTrigger.whileTrue(&autoFire)->onFalse(&lookJoystick);
        autoDriveTrigger.onTrue(&odoPointForwards)->onTrue(&autoDrive);//->onTrue(&testMoveCommand);
        // drive commands 

        // joystickDrive0.onTrue(&initialMoveCommand);
        joystickDrive1.onTrue(&drivetrainFollowJoystick)->onTrue(&lookJoystick)->onTrue(&odoPointForwards);
        joystickDrive2.onTrue(&beybladeJoystick)->onTrue(&lookJoystick)->onTrue(&odoPointForwards);
        
        isStopped = false;
    }

    
    bool startAdvance = false;
    bool startRetreat = false;
    int count = 0;
    void update() override {
        
        if (autoDriveTrigger.getAsBoolean() && drivers->refSerial.getRefSerialReceivingData() && 
        (drivers->refSerial.getGameData().gameType == RefSerialData::Rx::GameType::ROBOMASTER_RMUL_3V3)) {
       if (drivers->refSerial.getGameData().gameStage == RefSerialData::Rx::GameStage::IN_GAME) {
            // allow both
            if(!startAdvance && drivers->refSerial.getRobotData().currentHp > 390){
                startAdvance = true;
                startRetreat = false; // stop retreating
                drivers->commandScheduler.addCommand(&initialMoveCommand);
            }

            if(!startRetreat && drivers->refSerial.getRobotData().currentHp <= 200){
                startRetreat = true;
                startAdvance = false; // stop advancing
                drivers->commandScheduler.addCommand(&retreatMoveCommand);
            }
            // if(drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP){
            //     count = 10000;
            //     // drivers->commandScheduler.addCommand(&initialMoveCommand);
            }
       }
    //  count--;
    // }
    // else {
    //     count = 10000; // reset count if not in auto drive mode
    // }


        
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
        update();
    }

    bool isStopped = true;

    src::Drivers* drivers;
    SentryHardware hardware;

    // subsystems
    subsystems::GimbalSubsystem gimbal{drivers, &hardware.yawMotor, &hardware.pitchMotor};
    subsystems::FlywheelSubsystem flywheel{drivers, &hardware.flywheelMotor1, &hardware.flywheelMotor2};
    subsystems::DoubleIndexerSubsystem indexer{drivers, &hardware.indexMotor1, &hardware.indexMotor2};
    subsystems::DrivetrainSubsystem drivetrain{drivers, &hardware.driveMotor1, &hardware.driveMotor2, &hardware.driveMotor3, &hardware.driveMotor4};
    subsystems::OdometrySubsystem odo{drivers, &hardware.odoMotor};
    subsystems::JetsonSubsystem jetson{drivers, &gimbal};

    // commands
    commands::JoystickMoveCommand lookJoystick{drivers, &gimbal};
    commands::GimbalStopCommand stopGimbal{drivers, &gimbal};
    commands::AutoDriveCommand autoDrive{drivers, &drivetrain, &gimbal, &jetson};
    commands::AutoAimAndFireCommand autoFire{drivers, &gimbal, &indexer, &flywheel, &jetson, &autoDrive};

    commands::ShooterStartCommand shooterStart{drivers, &flywheel};
    commands::ShooterStopCommand shooterStop{drivers, &flywheel};

    commands::IndexerNBallsCommand indexerStart{drivers, &indexer, -1, 10};
    commands::IndexerUnjamCommand indexerUnjam{drivers, &indexer};

    commands::IndexerStopCommand indexerStop{drivers, &indexer};

    commands::OdometryPointForwardsCommand odoPointForwards{drivers, &odo, &gimbal};
    commands::OdometryStopCommand odoStop{drivers, &odo};

    // CHANGE NUMBERS LATER
    commands::DrivetrainDriveCommand drivetrainFollowJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::FOLLOW_TURRET, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand beybladeJoystick{drivers, &drivetrain, &gimbal, commands::DriveMode::BEYBLADE, commands::ControlMode::CONTROLLER};
    commands::DrivetrainDriveCommand noSpinDriveCommand{drivers, &drivetrain, &gimbal, commands::DriveMode::NO_SPIN, commands::ControlMode::CONTROLLER};

    commands::DrivetrainStopCommand stopDriveCommand{drivers, &drivetrain};

    commands::MoveToPositionCommand m0{drivers, &drivetrain, &gimbal, Pose2d(-0.2f, 0.0f, 0.0f), 0.2f};
    commands::MoveToPositionCommand m1{drivers, &drivetrain, &gimbal, Pose2d(5.2f, 0.0f, 0.0f), 0.5f};
    commands::MoveToPositionCommand m2{drivers, &drivetrain, &gimbal, Pose2d(5.2f, 4.0f, 0.0f), 0.5f};
    commands::MoveToPositionCommand m3{drivers, &drivetrain, &gimbal, Pose2d(2.2f, 4.5f, 0.0f), 0.0f};//Pose2d(3.0f, 4.2f, 0.0f)};

    std::array<Command*, 3> moveCommands1 = {&m1, &m2, &m3};
    SequentialCommand<3> initialMoveCommand{moveCommands1};

    std::array<Command*, 4> moveCommands2 = {&m2, &m1, &m0, &autoDrive};
    SequentialCommand<4> retreatMoveCommand{moveCommands2};

    SequentialCommand<4> testMoveCommand{{&m1, &m2, &m1, &m0}};
    
    // mappings 

    // shooting
    Trigger shootButton{drivers, Remote::Channel::WHEEL, -0.5};
    Trigger unjamButton{drivers, Remote::Channel::WHEEL, 0.5};

    // controller driving
    Trigger joystickDrive0{
        drivers,
        Remote::Switch::LEFT_SWITCH,
        Remote::SwitchState::DOWN};  // = (Trigger(drivers, Remote::Key::Q) & Trigger(drivers, Remote::Key::E)) | Trigger(drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
    Trigger joystickDrive1{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID};
    Trigger joystickDrive2{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN};

    Trigger autoFireTrigger{drivers, Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP};
    Trigger autoDriveTrigger{drivers, Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP};

    Trigger stopFlywheelTrigger = unjamButton | !autoFireTrigger; //doesn't get added to the list of triggers, is special, during a match the only way to turn off flywheels is to turn off the remote

    Trigger* triggers[7] = {&joystickDrive0, &joystickDrive1, &joystickDrive2, &shootButton, &unjamButton, &autoFireTrigger, &autoDriveTrigger};  //, &indexSpinButton};
};
}  // namespace robots
