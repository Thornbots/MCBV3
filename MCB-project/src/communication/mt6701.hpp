#pragma once



#include "modm/architecture/interface/i2c_device.hpp"
#include "modm/math/geometry/angle.hpp"
#include "modm/processing/protothread/protothread.hpp"

namespace communication{
template <class I2cMaster>
class MT6701 : public modm::I2cDevice<I2cMaster, 1>, public modm::pt::Protothread {
public:
    MT6701() : modm::I2cDevice<I2cMaster, 1>(ADDRESS) {};

    bool run() {
        PT_BEGIN();
        while (true)
        {
            buffer[0] = uint8_t(ANGLE_ADDR);
            PT_WAIT_UNTIL(this->startWriteRead(buffer, 1, buffer, 1));
            PT_WAIT_WHILE(this->isTransactionRunning());

            if (this->wasTransactionSuccessful())
            {
                angle = buffer[0];
            }
            
        }
        PT_END();
    }

    float getAngle() { return 0.0000457891f*angle*angle - 0.0361289286f*angle + 3.4231706783f; }

    uint16_t getRawAngle() { return angle; }

private:
    uint16_t angle = 0;

    static const uint8_t ADDRESS = 0x06;
    static const uint8_t ANGLE_ADDR = 0x03;
    uint8_t buffer[2];
    bool online = false;
};
}