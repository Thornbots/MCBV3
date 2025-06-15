#pragma once

#include "tap/control/command.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/flywheel/FlywheelSubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/indexer/IndexerSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "subsystems/servo/ServoSubsystem.hpp"

#include "util/ui/GraphicsContainer.hpp"
#include "ChassisOrientationIndicator.hpp"
#include "LaneAssistLines.hpp"
#include "Reticle.hpp"
#include "SupercapChargeIndicator.hpp"
#include "PeekingLines.hpp"
#include "HopperLidIndicator.hpp"
#include "HitRing.hpp"
#include "PredictedRemainingShotsIndicator.hpp"
#include "AllRobotHealthNumbers.hpp"
#include "Countdown.hpp"
#include "LinearVelocityIndicator.hpp"
#include "drivers.hpp"

namespace commands {
using subsystems::UISubsystem;

class InfantryDrawCommand : public tap::control::Command, GraphicsContainer {
public:
    InfantryDrawCommand(tap::Drivers* drivers, UISubsystem* ui, GimbalSubsystem* gimbal, FlywheelSubsystem* flywheel, IndexerSubsystem* indexer, DrivetrainSubsystem* drivetrain, ServoSubsystem* servo)
        : drivers(drivers),
          ui(ui),
          gimbal(gimbal),
          flywheel(flywheel),
          indexer(indexer),
          drivetrain(drivetrain),
          servo(servo) {
        addSubsystemRequirement(ui);

        addGraphicsObject(&lane);
        addGraphicsObject(&supercap);
        addGraphicsObject(&orient);
        addGraphicsObject(&peek);
        addGraphicsObject(&lid);
        addGraphicsObject(&reticle);
        addGraphicsObject(&ring);
        addGraphicsObject(&remain);
        addGraphicsObject(&numbers);
        addGraphicsObject(&countdown);
        addGraphicsObject(&velo);
    };

    void initialize() override { ui->setTopLevelContainer(this); };

    void execute() override {
        lane.update();
        supercap.update();
        orient.update();
        peek.update();
        lid.update();
        reticle.update();
        ring.update();
        remain.update();
        numbers.update();
        countdown.update();
        velo.update();
    };

    //ui subsystem won't do anything until its top level container is set, so we are ok to add objects to the command in the constructor
    void end(bool) override { /*ui->setTopLevelContainer(nullptr);*/ };

    bool isFinished() const override { return false; };  // never done drawing ui

    const char* getName() const override { return "infantry ui draw command"; }

private:
    tap::Drivers* drivers;
    UISubsystem* ui; 
    GimbalSubsystem* gimbal;
    FlywheelSubsystem* flywheel;
    IndexerSubsystem* indexer;
    DrivetrainSubsystem* drivetrain;
    ServoSubsystem* servo;

    // add top level graphics objects here and in the constructor
    LaneAssistLines lane{gimbal};
    SupercapChargeIndicator supercap{drivetrain};
    ChassisOrientationIndicator orient{drivers, gimbal, drivetrain};
    PeekingLines peek{drivetrain, gimbal};
    HopperLidIndicator lid{servo};
    Reticle reticle{gimbal};
    HitRing ring{drivers, gimbal};
    PredictedRemainingShotsIndicator remain{drivers, indexer};
    AllRobotHealthNumbers numbers{drivers};
    Countdown countdown{drivers};
    LinearVelocityIndicator velo{drivetrain};
};
}  // namespace commands