#pragma once

#include "tap/drivers.hpp"
#include "modm/processing/protothread/protothread.hpp"
#include "communication/spi/spi_master_2.hpp"

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

#define CS_PIN GpioB12

namespace communication {

class SPICommunication {
public:

    


    SPICommunication(tap::Drivers* drivers) : drivers(drivers){
    }
    
    void initialize() {
        GpioInputB14::configure(Gpio::InputType::PullUp); //pullup MISO
        SpiMaster2::connect<GpioOutputB15::Mosi, GpioOutputB13::Sck, GpioInputB14::Miso>();
        SpiMaster2::initialize<Board::SystemClock, 1312500>(); //2625000, 1312500, 656250, 328125, 164062
        SpiMaster2::setDataMode(SpiMaster2::DataMode::Mode1);
        state=0;
        delayTimeout.restart(DELAY_AMT);
        CS_PIN::setOutput(modm::Gpio::High);
        
        // pushbutton on pwm1 testing
        // GpioE9::setInput();
        // GpioE9::configure(Gpio::InputType::PullUp);
    }

    void doTransfer() {
        // pushbutton on pwm1 testing
        // drivers->leds.set(tap::gpio::Leds::Green, GpioE9::read());
        
        if(state==0){
            if(delayTimeout.execute()){
            
                // do with esp
                setBuffers();
                // send out 12 bytes, read in 12 bytes
                CS_PIN::setOutput(modm::Gpio::Low);
                SpiMaster2::transferBlocking(sendBuffer, receiveBuffer, 12);
                CS_PIN::setOutput(modm::Gpio::High);
                
                drivers->leds.set(tap::gpio::Leds::Green, true);
                if(0!=strncmp((char*) receiveBuffer, "send to data", 12)) {
                    drivers->leds.set(tap::gpio::Leds::Green, false);
                }
                
                delayTimeout.restart(DELAY_AMT);
                state++;
            }
        } else if (state==1) {
            if(delayTimeout.execute()){
                drivers->leds.set(tap::gpio::Leds::Green, false);
                
                // send their response back
                CS_PIN::setOutput(modm::Gpio::Low);
                SpiMaster2::transferBlocking(sendBuffer, receiveBuffer, 12);
                CS_PIN::setOutput(modm::Gpio::High);
                
                delayTimeout.restart(DELAY_AMT);
                state=0; //go again
            }
        }
    }


private:
    tap::Drivers* drivers;
    tap::arch::MilliTimeout delayTimeout{};
    
    static constexpr int DELAY_AMT = 300;
    
    int8_t state = -1;
    uint8_t sendBuffer[13] {};
    uint8_t receiveBuffer[13] {};
    
    
    void setBuffers() {
        int i=0;
        receiveBuffer[i] = sendBuffer[i++] = 'd';
        receiveBuffer[i] = sendBuffer[i++] = 'a';
        receiveBuffer[i] = sendBuffer[i++] = 't';
        receiveBuffer[i] = sendBuffer[i++] = 'a';
        receiveBuffer[i] = sendBuffer[i++] = ' ';
        receiveBuffer[i] = sendBuffer[i++] = 't';
        receiveBuffer[i] = sendBuffer[i++] = 'o';
        receiveBuffer[i] = sendBuffer[i++] = ' ';
        receiveBuffer[i] = sendBuffer[i++] = 's';
        receiveBuffer[i] = sendBuffer[i++] = 'e';
        receiveBuffer[i] = sendBuffer[i++] = 'n';
        receiveBuffer[i] = sendBuffer[i++] = 'd';
    }
};

}  // namespace communication
