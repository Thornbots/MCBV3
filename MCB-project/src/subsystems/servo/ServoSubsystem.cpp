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
    if(servoTimer.execute()) servo->updateSendPwmRamp();
}

void ServoSubsystem::setTargetPosition(float position) {
    if(servoSecondTimer.execute()) servo->setTargetPwm(position); 
}


} //namespace subsystems