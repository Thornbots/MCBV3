#pragma once

#include "tap/communication/serial/ref_serial_data.hpp"


#define RefSerialData tap::communication::serial::RefSerialData

class HitTracker 
{
    public:
        float hitOrientation; // Hit Direction in Radians
        bool isHit = false;
        uint16_t previousHp;
        
        static constexpr float PLACEHOLDER_ANGLE = 123;  // a special value for telling jetson that you weren't hit



        HitTracker(tap::Drivers* drivers) : drivers(drivers)
        {
        }


        // Gimbal Function - Get Yaw Encoder
        // ------------------------------------------
        // Provide the ability to run the function without causing a circular dependency
        
        // Lambda Implementation
        void setGetYawEncoderFunction(std::function<float()> func)
        {
            this->getYawEncoderValue_func = func;
        }

        // Function to append a hit if the code structure already identifies a strike
        void addHit()
        {
            float encoder = getYawEncoderValue_func();
            float imu = drivers->bmi088.getYaw() / 180.0f * PI;

            // damagedArmorId==0 is forward, add 0*90 degrees
            // 1 is left, add 1*(pi/2) Radians
            // 2 is back, add 2*(pi/2) Radians
            // 3 is right, add 3*(pi/2) Radians
            // 4 is top, don't care because we don't have panels on top (yet?)
            
            hitOrientation = -encoder + imu + ((PI/2) * ((uint16_t)drivers->refSerial.getRobotData().damagedArmorId));
            isHit = true;
        }

        // Detect if a hit occurs -- appends the hit orientation
        void update() 
        {
            isHit = false;

            // check for a new hit
            if (drivers->refSerial.getRefSerialReceivingData() && getYawEncoderValue_func) {
                const RefSerialData::Rx::RobotData &robotData = drivers->refSerial.getRobotData();
                if (previousHp > robotData.currentHp && (robotData.damageType == RefSerialData::Rx::DamageType::ARMOR_DAMAGE || robotData.damageType == RefSerialData::Rx::DamageType::COLLISION)) {
                    // took some sort of damage and we think we took panel damage
                    addHit();
                }

                previousHp = robotData.currentHp;
            }
        }
 
        // Return angle in radians for the sentry to turn to
        float getAngleToTurnForSentry() {
            if (isHit) return hitOrientation;

            return PLACEHOLDER_ANGLE;
        }

    private:
        tap::Drivers* drivers;

        // Access gimbal yaw encoder value without causing circular dependency
        std::function<float()> getYawEncoderValue_func;
};
