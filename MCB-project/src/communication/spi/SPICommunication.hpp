#pragma once

#include "tap/drivers.hpp"

#include "communication/spi/spi_master_2.hpp"
#include "modm/processing/protothread/protothread.hpp"

// type c to turret raspberry pi pico connection

// Picos hold the type c's MISO pin low when the
// cs to that pico tells it its not supposed to
// communicate right now. This means that
// multiple picos can't talk to the one type c
// even though the type c has multiple pins
// that can work as cs pins.

// B12 | CS in the 2x4 'Customizable I/O Port', top right corner
// E9  | PWM pin 1, the pin closest to the 'PWM' text on the type c
// E11 | PWM pin 2, next to pwm pin 1 across the thin wall
// E13 | PWM pin 3
// E14 | PWM pin 4
// C6  | PWM pin 5
// I6  | PWM pin 6
// I7  | PWM pin 7, the one next to DBUS (dbus is used for white controller receiver)

#define CS_PIN GpioB12

namespace communication {
    
// May want a bit wasCorrectlyReceived so that both can know if one way of the comms is messed up
struct spiTypeCToPicoStruct {
    int32_t a=0;
    int32_t b=0;
} modm_packed;

struct spiPicoToTypeCStruct {
    float yawPosition=0; // radians
    float yawVelocity=0; // radians/second
} modm_packed;

// the way spi works, sending and receiving must be the same number of bytes
// you may need to add something like uint8_t unused[4] in the future to one of the structs
static_assert(sizeof(struct spiTypeCToPicoStruct) == sizeof(struct spiPicoToTypeCStruct));

template <typename msg_type>
struct spiDataFrame {
    uint16_t dataLen; //bytes, for catching when type c and pico don't agree on the size of spiTypeCToPicoStruct/spiPicoToTypeCStruct because one was updated before the other, also catches if communcications are all 0's or all 1's
    msg_type data;
    uint16_t crc16;

    // for making this spiDataFrame ready to send on the spi connection
    spiDataFrame(const msg_type&& inputData) {
        dataLen = sizeof(msg_type);
        data=inputData;
        // std::memcpy(&data, inputData, sizeof(msg_type));
        crc16 = calculateCrc16();
    }
    spiDataFrame() {}
    
    // for when this spiDataFrame is data received from the spi connection
    bool isCorrectlyReceived() {
        return dataLen == sizeof(msg_type) && crc16 == calculateCrc16();
    }
    
    uint16_t calculateCrc16() {
        // crc16 over everything (except the crc16 of course)
        return tap::algorithms::calculateCRC16(reinterpret_cast<uint8_t*>(this), offsetof(struct spiDataFrame<msg_type>, crc16));
    }
} modm_packed;

class SPICommunication {
public:

    SPICommunication(tap::Drivers* drivers) : drivers(drivers) {}

    void initialize() {
        GpioInputB14::configure(Gpio::InputType::PullUp);  // pullup MISO
        SpiMaster2::connect<GpioOutputB15::Mosi, GpioOutputB13::Sck, GpioInputB14::Miso>();
        SpiMaster2::initialize<Board::SystemClock, 1312500>();  // 2625000, 1312500, 656250, 328125, 164062
        SpiMaster2::setDataMode(SpiMaster2::DataMode::Mode1);
        state = 0;
        delayTimeout.restart(DELAY_AMT);
        CS_PIN::setOutput(modm::Gpio::High);
    }

    void doTransfer() {
        if (state == 0 && delayTimeout.execute()) { //waiting for pico startup
            state++;
        } else if (state == 1) {
            struct spiDataFrame<spiTypeCToPicoStruct> toSend{{
                .a=5,
                .b=4
            }};
            struct spiDataFrame<spiPicoToTypeCStruct> received;
            
            CS_PIN::setOutput(modm::Gpio::Low);
            SpiMaster2::transferBlocking(reinterpret_cast<uint8_t*>(&toSend), reinterpret_cast<uint8_t*>(&received), sizeof(toSend));
            CS_PIN::setOutput(modm::Gpio::High);
            
            drivers->leds.set(tap::gpio::Leds::Green, received.isCorrectlyReceived() && received.data.yawPosition>0);
        }
    }

private:
    tap::Drivers* drivers;
    tap::arch::MilliTimeout delayTimeout{};

    static constexpr int DELAY_AMT = 300;

    int8_t state = -1;
};

}  // namespace communication
