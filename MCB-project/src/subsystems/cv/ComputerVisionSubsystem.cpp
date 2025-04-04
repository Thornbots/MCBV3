#include "ComputerVisionSubsystem.hpp"
// #include "ComputerVisionSubsystemConstants.hpp"

namespace subsystems {

ComputerVisionSubsystem::ComputerVisionSubsystem(tap::Drivers* drivers) : tap::control::Subsystem(drivers), comm(drivers, tap::communication::serial::Uart::UartPort::Uart1, true) {}

void ComputerVisionSubsystem::initialize() {
    drivers->commandScheduler.registerSubsystem(this);
    comm.initialize();
    drivers->leds.init();
}

void ComputerVisionSubsystem::refresh() { 
    comm.update(); 
    drivers->leds.set(tap::gpio::Leds::Red, comm.getLastCVData()!=nullptr);
}

}  // namespace subsystems