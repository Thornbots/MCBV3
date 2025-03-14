#include "ShooterStartCommand.hpp"

namespace commands
{

void ShooterStartCommand::initialize() {  }
void ShooterStartCommand::execute()
{
    flywheel->setTargetVelocity(flywheel->FLYWHEEL_MOTOR_MAX_RPM);
}

void ShooterStartCommand::end(bool) {}

// doesn't have a real end condition
bool ShooterStartCommand::isFinished(void) const { return false; }
}  // namespace commands