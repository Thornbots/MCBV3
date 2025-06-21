#pragma once

#include "tap/control/command.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/flywheel/FlywheelSubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/indexer/HeroIndexerSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"

#include "util/ui/GraphicsContainer.hpp"
#include "ChassisOrientationIndicator.hpp"
#include "LaneAssistLines.hpp"
#include "Reticle.hpp"
#include "SupercapChargeIndicator.hpp"
#include "HitRing.hpp"
#include "PredictedRemainingShotsIndicator.hpp"
#include "AllRobotHealthNumbers.hpp"
#include "Countdown.hpp"
#include "LinearVelocityIndicator.hpp"
#include "ImuRecalibrationIndicator.hpp"
#include "drivers.hpp"

namespace commands {
using subsystems::UISubsystem;

class HeroDrawCommand : public tap::control::Command, GraphicsContainer {
public:
    HeroDrawCommand(src::Drivers* drivers, UISubsystem* ui, GimbalSubsystem* gimbal, FlywheelSubsystem* flywheel, HeroIndexerSubsystem* indexer, DrivetrainSubsystem* drivetrain)
        : drivers(drivers),
          ui(ui),
          gimbal(gimbal),
          flywheel(flywheel),
          indexer(indexer),
          drivetrain(drivetrain) {
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

    // add top level graphics objects here and in the constructor
    LaneAssistLines lane{gimbal};
    SupercapChargeIndicator supercap{drivetrain};
    ChassisOrientationIndicator orient{drivers, gimbal, drivetrain};
    Reticle reticle{gimbal};
    HitRing ring{drivers, gimbal};
    PredictedRemainingShotsIndicator remain{drivers, indexer};
    AllRobotHealthNumbers numbers{drivers};
    Countdown countdown{drivers};
    LinearVelocityIndicator velo{drivetrain};
    ImuRecalibrationIndicator recal{drivers};
};
}  // namespace commands