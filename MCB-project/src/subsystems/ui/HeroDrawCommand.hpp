#pragma once

#include "tap/control/command.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/flywheel/FlywheelSubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/indexer/HeroIndexerSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"

#include "util/ui/GraphicsContainer.hpp"
#include "objects/ChassisOrientationIndicator.hpp"
#include "objects/LaneAssistLines.hpp"
#include "objects/Reticle.hpp"
#include "objects/SupercapChargeIndicator.hpp"
#include "objects/HitRing.hpp"
#include "objects/PredictedRemainingShotsIndicator.hpp"
#include "objects/AllRobotHealthNumbers.hpp"
#include "objects/Countdown.hpp"
#include "objects/LinearVelocityIndicator.hpp"
#include "objects/ImuRecalibrationIndicator.hpp"
#include "objects/TorqueIndicator.hpp"
#include "drivers.hpp"

namespace commands {
using subsystems::UISubsystem;

class HeroDrawCommand : public tap::control::Command, GraphicsContainer {
public:
    HeroDrawCommand(src::Drivers* drivers, UISubsystem* ui, GimbalSubsystem* gimbal, FlywheelSubsystem* flywheel, HeroIndexerSubsystem* indexer, DrivetrainSubsystem* drivetrain, tap::motor::DjiMotor* motorFlywheel1, tap::motor::DjiMotor* motorFlywheel2, tap::motor::DjiMotor* motorFlywheel3, tap::motor::DjiMotor* motorFlywheel4)
        : drivers(drivers),
          ui(ui),
          gimbal(gimbal),
          flywheel(flywheel),
          indexer(indexer),
          drivetrain(drivetrain),
          motorFlywheel1(motorFlywheel1),
          motorFlywheel2(motorFlywheel2),
          motorFlywheel3(motorFlywheel3),
          motorFlywheel4(motorFlywheel4) {
        addSubsystemRequirement(ui);

        addGraphicsObject(&lane);
        addGraphicsObject(&supercap);
        addGraphicsObject(&orient);
        addGraphicsObject(&reticle);
        addGraphicsObject(&ring);
        addGraphicsObject(&remain);
        addGraphicsObject(&numbers);
        addGraphicsObject(&countdown);
        addGraphicsObject(&velo);
        addGraphicsObject(&recal);
        addGraphicsObject(&ind1);
        addGraphicsObject(&ind2);
        addGraphicsObject(&ind3);
        addGraphicsObject(&ind4);
    };

    void initialize() override { ui->setTopLevelContainer(this); };

    void execute() override {
        lane.update();
        supercap.update();
        orient.update();
        reticle.update();
        ring.update();
        remain.update();
        numbers.update();
        countdown.update();
        velo.update();
        recal.update();
        ind1.update();
        ind2.update();
        ind3.update();
        ind4.update();
    };

    //ui subsystem won't do anything until its top level container is set, so we are ok to add objects to the command in the constructor
    void end(bool) override { /*ui->setTopLevelContainer(nullptr);*/ };

    bool isFinished() const override { return false; };  // never done drawing ui

    const char* getName() const override { return "hero ui draw command"; }

private:
    src::Drivers* drivers;
    UISubsystem* ui;
    GimbalSubsystem* gimbal;
    FlywheelSubsystem* flywheel;
    HeroIndexerSubsystem* indexer;
    DrivetrainSubsystem* drivetrain;
    tap::motor::DjiMotor* motorFlywheel1;
    tap::motor::DjiMotor* motorFlywheel2;
    tap::motor::DjiMotor* motorFlywheel3;
    tap::motor::DjiMotor* motorFlywheel4;

    // add top level graphics objects here and in the constructor
    LaneAssistLines lane{gimbal};
    SupercapChargeIndicator supercap{drivetrain};
    ChassisOrientationIndicator orient{true, drivers, gimbal, drivetrain};
    Reticle reticle{drivers, gimbal, indexer};
    HitRing ring{drivers, gimbal};
    PredictedRemainingShotsIndicator remain{drivers, indexer};
    AllRobotHealthNumbers numbers{drivers};
    Countdown countdown{drivers};
    LinearVelocityIndicator velo{drivetrain};
    ImuRecalibrationIndicator recal{drivers};
    TorqueIndicator ind1{drivers, motorFlywheel1, 1};
    TorqueIndicator ind2{drivers, motorFlywheel2, 2};
    TorqueIndicator ind3{drivers, motorFlywheel3, 3};
    TorqueIndicator ind4{drivers, motorFlywheel4, 4};
};
}  // namespace commands