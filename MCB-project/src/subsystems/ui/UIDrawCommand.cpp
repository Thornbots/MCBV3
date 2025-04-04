#include "UIDrawCommand.hpp"

// this cpp file doesn't have much, might move stuff in here to the hpp and not have a cpp
namespace commands {

UIDrawCommand::UIDrawCommand(UISubsystem* ui, GimbalSubsystem* gimbal, FlywheelSubsystem* flywheel, IndexerSubsystem* indexer, DrivetrainSubsystem* drivetrain)
    : ui(ui),
      gimbal(gimbal),
      flywheel(flywheel),
      indexer(indexer),
      drivetrain(drivetrain),
      laneAssistLines(gimbal) {
    addSubsystemRequirement(ui);
    
    // addGraphicsObject(&testGraphics);
    // addGraphicsObject(&testFill);
    addGraphicsObject(&laneAssistLines);
    addGraphicsObject(&supercapChargeIndicator);
    addGraphicsObject(&chassisOrientationIndicator);

    chassisOrientationIndicator.setGimbalSubsystem(gimbal);
}

void UIDrawCommand::initialize() { ui->setTopLevelContainer(this); }

void UIDrawCommand::execute() {}

void UIDrawCommand::end(bool) { ui->setTopLevelContainer(nullptr); }

bool UIDrawCommand::isFinished(void) const { return false; }
}  // namespace commands