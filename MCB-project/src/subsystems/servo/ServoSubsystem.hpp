#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"
#include "tap/control/subsystem.hpp"

#include "drivers.hpp"

namespace subsystems
{
static tap::arch::PeriodicMilliTimer servoTimer(20);
static tap::arch::PeriodicMilliTimer servoSecondTimer(100);
class ServoSubsystem : public tap::control::Subsystem
{

protected:  // Private Variables
src::Drivers* drivers;
tap::motor::Servo* servo;

public:  // Public Methods

constexpr static float OPEN_POSITION = 1.0f;
constexpr static float CLOSED_POSITION = 0.0f;

ServoSubsystem(src::Drivers* drivers, tap::motor::Servo* servo);

~ServoSubsystem() {}

void initialize();

void refresh() override;

virtual void setTargetPosition(float position);

private:  // Private Methods
};
} //namespace subsystems