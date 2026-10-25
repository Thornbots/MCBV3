#pragma once

#include "tap/drivers.hpp"
#include "modm/processing/protothread/protothread.hpp"
#include "communication/spi/spi_master_2.hpp"

//right now, the resumable function version of transfer doesn't work properly

// type c to turret raspberry pi pico connection

// Picos hold the type c's MISO pin low when the
// cs to that pico tells it its not supposed to 
// communicate right now. This means that
// multiple picos can't talk to the one type c
// even though the type c has multiple pins
// that can work as cs pins.

//B12 | CS in the 2x4 'Customizable I/O Port', top right corner
//E9  | PWM pin 1, the pin closest to the 'PWM' text on the type c
//E11 | PWM pin 2, next to pwm pin 1 across the thin wall
//E13 | PWM pin 3
//E14 | PWM pin 4
//C6  | PWM pin 5
//I6  | PWM pin 6
//I7  | PWM pin 7, the one next to DBUS (dbus is used for white controller receiver)

#define CS_PIN GpioOutputB12

namespace communication {

class SPICommunicationPT : public modm::pt::Protothread {
public:

    SPICommunicationPT(tap::Drivers* drivers) : drivers(drivers){
        
    }
    
    void initialize() {
        GpioInputB14::configure(Gpio::InputType::PullUp); //pullup MISO
        SpiMaster2::connect<GpioOutputB15::Mosi, GpioOutputB13::Sck, GpioInputB14::Miso>();
        SpiMaster2::initialize<Board::SystemClock, 1312500>(); //2625000, 1312500, 656250, 328125, 164062
        SpiMaster2::setDataMode(SpiMaster2::DataMode::Mode1);
        initialized = true;
    }

    bool run() {
        PT_BEGIN();
        
        PT_WAIT_UNTIL(initialized);
        
        while (true) {
            // send out 12 bytes, read in 12 bytes
            CS_PIN::setOutput(modm::Gpio::Low);
            PT_CALL(SpiMaster2::transfer(sendBuffer, receiveBuffer, 12)); //right now, the resumable function version of transfer doesn't work properly
            CS_PIN::setOutput(modm::Gpio::High);
            
            drivers->leds.set(tap::gpio::Leds::Green, true);
            if(0!=strncmp((char*) receiveBuffer, "send to data", 12)) {
                // turn the green led on for 3 seconds before it turns blue to recal imu
                drivers->leds.set(tap::gpio::Leds::Green, false);
            }
            
            delayTimeout.restart(3000);
            PT_WAIT_UNTIL(delayTimeout.execute());
            drivers->leds.set(tap::gpio::Leds::Green, false);
            
            // send their response back
            CS_PIN::setOutput(modm::Gpio::Low);
            PT_CALL(SpiMaster2::transfer(receiveBuffer, sendBuffer, 12));
            CS_PIN::setOutput(modm::Gpio::High);
        }
        
        PT_END();
    }


private:
    tap::Drivers* drivers;
    tap::arch::MilliTimeout delayTimeout{};
    bool initialized = false;
    
    bool shouldBeGreen = false;
    
    uint8_t sendBuffer[13] { "data to send" };
    uint8_t receiveBuffer[13] {"abc"};
};

}  // namespace communication
