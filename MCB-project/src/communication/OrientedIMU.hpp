#pragma once

#include "tap/drivers.hpp"

namespace communication {

class OrientedIMU
{
public:
    OrientedIMU(tap::Drivers *drivers) : drivers(drivers) {

    }

    float getRoll() {
        return drivers->bmi088.getRoll();
    }

    float getPitch() {
        return drivers->bmi088.getPitch();
    }

    float getYaw() {
        return drivers->bmi088.getYaw();
    }



    float getGx() {
        #if defined(HERO)
            return -drivers->bmi088.getGz();
        #else
            return drivers->bmi088.getGx();
        #endif
    }

    float getGy() {
        return drivers->bmi088.getGy();
    }

    float getGz() {
        #if defined(HERO)
            return drivers->bmi088.getGx();
        #else
            return drivers->bmi088.getGz();
        #endif
    }



    float getq0(){
        return drivers->bmi088.getq0();
    }

    float getq1(){
        return drivers->bmi088.getq1();
    }

    float getq2(){
        return drivers->bmi088.getq2();
    }

    float getq3(){
        return drivers->bmi088.getq3();
    }


private:
    tap::Drivers *drivers;

};

} // namespace communication
