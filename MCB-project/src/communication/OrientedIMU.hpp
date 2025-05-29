#pragma once

#include "tap/drivers.hpp"

namespace communication {

class OrientedIMU
{
public:
    OrientedIMU(tap::Drivers *drivers) : drivers(drivers) {

    }

    float getRoll() {
        #if defined(HERO)
            return -drivers->bmi088.getYaw();
        #else
            return drivers->bmi088.getRoll();
        #endif
    }

    float getPitch() {
        return drivers->bmi088.getPitch();
    }

    float getYaw() {
        #if defined(HERO)
            return drivers->bmi088.getRoll();
        #else
            return drivers->bmi088.getYaw();
        #endif
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
        return o0*drivers->bmi088.getq0() - o1*drivers->bmi088.getq1() - o2*drivers->bmi088.getq2() - o3*drivers->bmi088.getq3();
    }

    float getq1(){
        return o0*drivers->bmi088.getq1() + o1*drivers->bmi088.getq0() + o2*drivers->bmi088.getq3() - o3*drivers->bmi088.getq2();
    }

    float getq2(){
        return o0*drivers->bmi088.getq2() + o2*drivers->bmi088.getq0() + o3*drivers->bmi088.getq1() - o1*drivers->bmi088.getq3();
    }

    float getq3(){
        return o0*drivers->bmi088.getq3() + o3*drivers->bmi088.getq0() + o1*drivers->bmi088.getq2() - o2*drivers->bmi088.getq1();
    }


    void setOrientationQuaternion(float q0, float q1, float q2, float q3){
        this->o0 = q0;
        this->o1 = q1;
        this->o2 = q2;
        this->o3 = q3;
    }

private:
    tap::Drivers *drivers;

    float o0 = 1;
    float o1 = 0;
    float o2 = 0;
    float o3 = 0;

};

} // namespace communication
