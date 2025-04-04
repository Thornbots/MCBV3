#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"
#include "tap/control/subsystem.hpp"
#include "communication/JetsonCommunication.hpp"


#include "drivers.hpp"

using namespace communication;

namespace subsystems
{

class ComputerVisionSubsystem : public tap::control::Subsystem
{

private:   // Private Variables

JetsonCommunication comm;

public:   // Public Methods

ComputerVisionSubsystem(tap::Drivers* drivers);

~ComputerVisionSubsystem() {}

void initialize();

void refresh() override;


private:  // Private Methods
};
} //namespace subsystems