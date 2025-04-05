#include "ServoSubsystem.hpp"

namespace subsystems {
    
ServoSubsystem::ServoSubsystem(src::Drivers* drivers, tap::motor::Servo* servo)
    : tap::control::Subsystem(drivers),
    drivers(drivers),
    servo(servo)
    {}

void ServoSubsystem::initialize() {
    drivers->commandScheduler.registerSubsystem(this);
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER1, 333); 
}

void ServoSubsystem::refresh() {
    servo->updateSendPwmRamp();
}

void ServoSubsystem::setTargetPosition(float position) {
    servo->setTargetPwm(position);
}


} //namespace subsystems