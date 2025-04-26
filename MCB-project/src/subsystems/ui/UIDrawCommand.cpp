#include "UIDrawCommand.hpp"

// this cpp file doesn't have much, might move stuff in here to the hpp and not have a cpp
namespace commands {

UIDrawCommand::UIDrawCommand(UISubsystem* ui, GimbalSubsystem* gimbal, FlywheelSubsystem* flywheel, IndexerSubsystem* indexer, DrivetrainSubsystem* drivetrain)
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

    laneAssistLines.setGimbalSubsystem(gimbal);
    chassisOrientationIndicator.setGimbalSubsystem(gimbal);
}

void UIDrawCommand::initialize() { ui->setTopLevelContainer(this); }

void UIDrawCommand::execute() {
  laneAssistLines.update();
  chassisOrientationIndicator.update();
}

void UIDrawCommand::end(bool) { ui->setTopLevelContainer(nullptr); }

bool UIDrawCommand::isFinished(void) const { return false; }
}  // namespace commands