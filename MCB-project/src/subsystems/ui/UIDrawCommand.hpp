#pragma once

#include "tap/control/command.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/flywheel/FlywheelSubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/indexer/IndexerSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"

#include "ChassisOrientationIndicator.hpp"
#include "LaneAssistLines.hpp"
#include "Reticle.hpp"
#include "SupercapChargeIndicator.hpp"
#include "PeekingLines.hpp"
#include "TestFill.hpp"
#include "TestGraphics.hpp"
#include "drivers.hpp"

namespace commands {
using subsystems::UISubsystem;

class UIDrawCommand : public tap::control::Command, GraphicsContainer {
public:
    UIDrawCommand(UISubsystem* ui, GimbalSubsystem* gimbal, FlywheelSubsystem* flywheel, IndexerSubsystem* indexer, DrivetrainSubsystem* drivetrain)
        : ui(ui),
          gimbal(gimbal),
          flywheel(flywheel),
          indexer(indexer),
          drivetrain(drivetrain) {
        addSubsystemRequirement(ui);

        // addGraphicsObject(&testGraphics);
        // addGraphicsObject(&testFill);
        addGraphicsObject(&laneAssistLines);
        addGraphicsObject(&supercapChargeIndicator);
        addGraphicsObject(&chassisOrientationIndicator);
        addGraphicsObject(&peekingLines);

    };

    void initialize() override { ui->setTopLevelContainer(this); };

    void execute() override {
        laneAssistLines.update();
        supercapChargeIndicator.update();
        chassisOrientationIndicator.update();
        peekingLines.update();
    };

    //ui subsystem won't do anything until its top level container is set, so we are ok to add objects to the command in the constructor
    void end(bool) override { ui->setTopLevelContainer(nullptr); };

    bool isFinished() const override { return false; };  // never done drawing ui

    const char* getName() const override { return "ui draw command"; }

private:
    UISubsystem* ui; 
    GimbalSubsystem* gimbal;
    FlywheelSubsystem* flywheel;
    IndexerSubsystem* indexer;
    DrivetrainSubsystem* drivetrain;

    // add top level graphics objects here and in the constructor
    // TestGraphics testGraphics{};
    // TestFill testFill{};
    LaneAssistLines laneAssistLines{gimbal};
    SupercapChargeIndicator supercapChargeIndicator{drivetrain};
    ChassisOrientationIndicator chassisOrientationIndicator{gimbal};
    PeekingLines peekingLines{drivetrain};
};
}  // namespace commands