#pragma once

#include "modm/architecture/interface/i2c_device.hpp"
#include "modm/math/geometry/angle.hpp"
#include "modm/processing/protothread/protothread.hpp"
#include "tap/architecture/clock.hpp"

namespace communication {
template <class I2cMaster>
class LocalizationPico : public modm::I2cDevice<I2cMaster, 1>, public modm::pt::Protothread {
public:
    LocalizationPico() : modm::I2cDevice<I2cMaster, 1>(ADDRESS) {};

    bool run() {
        PT_BEGIN();
        // auto rez = this->ping();
        while (true) {
            // modm refuses to attach an Error transaction until explicitly reset.
            this->transaction.resetState();
            buffer2[0] = uint8_t(static_cast<uint8_t>(odom_x)); //uint8_t(ANGLE_ADDR);

            waitSinceMs = tap::arch::clock::getTimeMilliseconds();
            PT_WAIT_UNTIL(this->startWriteRead(buffer2, 1, (uint8_t*)buffer, sizeof(float)*4) || expired());
            if (expired()) {
                requestRecovery();
                PT_RESTART();
            }
            PT_WAIT_UNTIL(!this->isTransactionRunning() || expired());
            if (this->isTransactionRunning()) {
                requestRecovery();
                PT_RESTART();
            }

            success = this->wasTransactionSuccessful();
            if (success) {
                lastSuccessMs = tap::arch::clock::getTimeMilliseconds();
                everSucceeded = true;
                // s.printf("i2c result: %d %d %d %d\n", buffer[0], buffer[1], buffer[2], buffer[3]); 
                // float *rez = (float*)buffer;
                // static_cast<float>(buffer);

                // s.printf("odom:\n");
                // s.printf("odom: %f %f %f %f\n", buffer[0], buffer[1], buffer[2], buffer[3]); 
                odom_x = buffer[0]; 
                odom_y = buffer[1];
                odom_x_vel = buffer[2];
                odom_y_vel = buffer[3];
                // s.printf("odom: %f\n", this->getX());
                

                // angle <13:6>             | <5:0>
                // start 0b0000000011111111 | 0b0000000011111100
                //  end  0b0011111111000000 | 0b0000000000111111

                // angle = (static_cast<uint16_t>(buffer[0]) << 6);

                // buffer[0] = uint8_t(69);
                // PT_WAIT_UNTIL(this->startWriteRead(buffer, 1,buffer,4));
                // PT_WAIT_WHILE(this->isTransactionRunning());

                // if (this->wasTransactionSuccessful()) {
            
                //     angle |= (static_cast<uint16_t>(buffer[0]) >> 2);
                // }
            }
        }
        PT_END();
    }
    // i dont like this but it kinda works for old read. NEED TO CHANGE
    // float getAngle() { return 0.0000457891f * angle * angle - 0.0361289286f * angle + 3.4231706783f; }
    float getX() { return (odom_x + offsetX); }  //wheel diameter changed, need to update pi pico code
    float getY() { return (odom_y + offsetY); }  
    float getXVel() { return odom_x_vel; }  
    float getYVel() { return odom_y_vel; }  
    bool isFresh(uint32_t nowMs, uint32_t maxAgeMs) const { return everSucceeded && nowMs - lastSuccessMs < maxAgeMs; }
    
    float offsetX = 0;
    float offsetY = 0;

private:
    static constexpr uint32_t TRANSACTION_TIMEOUT_MS = 10;
    uint32_t waitSinceMs = 0;
    bool expired() const {
        return tap::arch::clock::getTimeMilliseconds() - waitSinceMs >= TRANSACTION_TIMEOUT_MS;
    }
    // refresh() resets/detaches the shared master before the next run().
    void requestRecovery() {
        I2cMaster::needsReinit = true;
    }
    uint32_t lastSuccessMs = 0;
    bool everSucceeded = false;
    bool success = false;
    float odom_x = 0;
    float odom_y = 0;
    float odom_x_vel = 0;
    float odom_y_vel = 0;

    static const uint8_t ADDRESS = 0x55;
    static const uint8_t ANGLE_ADDR = 0x55;

    float buffer[4]{}; //TODO: make buffer type float[4]

    uint8_t buffer2[4];
    bool online = false;
};
}  // namespace communication