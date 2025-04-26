#pragma once

#include "tap/control/command.hpp"

#include "subsystems/ui/UISubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/flywheel/FlywheelSubsystem.hpp"
#include "subsystems/indexer/IndexerSubsystem.hpp"
#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"

#include "LaneAssistLines.hpp"
#include "Reticle.hpp"
#include "SupercapChargeIndicator.hpp"
#include "ChassisOrientationIndicator.hpp"
#include "TestGraphics.hpp"
#include "TestFill.hpp"
#include "drivers.hpp"

namespace commands {
using subsystems::UISubsystem;

class UIDrawCommand : public tap::control::Command, GraphicsContainer {
public:
    UIDrawCommand(UISubsystem* ui, GimbalSubsystem* gimbal, FlywheelSubsystem* flywheel, IndexerSubsystem* indexer, DrivetrainSubsystem* drivetrain);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

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
    LaneAssistLines laneAssistLines{};
    SupercapChargeIndicator supercapChargeIndicator{};
    ChassisOrientationIndicator chassisOrientationIndicator{};
};
}  // namespace commands