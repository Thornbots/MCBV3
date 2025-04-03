#pragma once

#include "modm/architecture/interface/i2c_device.hpp"
#include "modm/math/geometry/angle.hpp"
#include "modm/processing/protothread/protothread.hpp"

template <class I2cMaster>
class MT6701 : public modm::I2cDevice<I2cMaster, 1>, public modm::pt::Protothread {
public:
    MT6701() : modm::I2cDevice<I2cMaster, 1>(ADDRESS) {};

    bool run() {
        PT_BEGIN();

        while (true) {
            buffer[0] = ANGLE_ADDR;  // this is the location of the angle

            PT_WAIT_UNTIL(this->startWriteRead(buffer, 1, buffer, 2));
            PT_WAIT_WHILE(this->isTransactionRunning());

            if (this->wasTransactionSuccessful()) {
                angle = (buffer[0] << 8) | buffer[1];
                online = true;
            }
        }

        PT_END();
    }

    float getAngle() { return angle / 16384.0f * M_TWOPI; }

    bool isOnline() { return online; }

protected:
    enum class Register : uint8_t { ANGLE = 0x03 };

private:
    static const uint8_t ADDRESS = 0x06;    
    static const uint8_t ANGLE_ADDR = 0x03;
    uint16_t angle = 0;
    uint8_t buffer[2];
    bool online = false;
};
