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
    if(servoSecondTimer.execute()) servo->setTargetPwm(targetIsClosed ? CLOSED_POSITION : OPEN_POSITION); 
}

void ServoSubsystem::setClosed() {
    targetIsClosed = true;
}
void ServoSubsystem::setOpen() {
    targetIsClosed = false;
}

bool ServoSubsystem::getTargetIsClosed() {
    return targetIsClosed;
}


} //namespace subsystems