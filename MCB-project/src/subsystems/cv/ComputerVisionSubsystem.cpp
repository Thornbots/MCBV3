#include "ComputerVisionSubsystem.hpp"
// #include "ComputerVisionSubsystemConstants.hpp"

namespace subsystems {

ComputerVisionSubsystem::ComputerVisionSubsystem(tap::Drivers* drivers) :
     tap::control::Subsystem(drivers), comm(drivers, tap::communication::serial::Uart::Uart1, true) {}

void ComputerVisionSubsystem::initialize() {
    drivers->commandScheduler.registerSubsystem(this);
    drivers->leds.init();
    comm.initialize();
}

void ComputerVisionSubsystem::refresh() { 
    comm.update(); 
    // drivers->leds.set(tap::gpio::Leds::Green, comm.getLastCVData()!=nullptr);
}

}  // namespace subsystems