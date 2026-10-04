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
            // 
        }


        // Gimbal Function - Get Yaw Encoder
        // ------------------------------------------
        // Provide the ability to run the function without causing a circular dependency
        
        // Lambda Implementation
        void setGetYawEncoderFunction(std::function<float()> func)
        {
            this->getYawEncoderValue_func = func;
        }

        // ------------------------------------------




        // Hit Detection
        // ------------------------------------------
        // Function to append a hit if the code structure already identifies a strike
        void addHit()
        {
            // Check if get yaw encoder function pointer is defined
            if (!getYawEncoderValue_func)
            {return;}
            
           

            float encoder = getYawEncoderValue_func();
            float imu = drivers->bmi088.getYaw() / 180 * PI;

            // damagedArmorId==0 is forward, add 0*90 degrees
            // 1 is left, add 1*(pi/2) Radians
            // 2 is back, add 2*(pi/2) Radians
            // 3 is right, add 3*(pi/2) Radians
            // 4 is top, don't care because we don't have panels on top (yet?)
            
            hitOrientation = -encoder + imu + (PI/2) * ((uint16_t)drivers->refSerial.getRobotData().damagedArmorId);
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
                    isHit = true;
                }

                previousHp = robotData.currentHp;
            }
        }
        // ------------------------------------------


 
        // Return Value Functions
        // ------------------------------------------
        // Return angle in radians for the sentry to turn to
        float getAngleToTurnForSentry() {
            if (isHit == false)
            {return PLACEHOLDER_ANGLE;}

            if(hitOrientation>PI) hitOrientation-=2*PI;
            return hitOrientation;
        }

        // ------------------------------------------


    


    private:
        tap::Drivers* drivers;
        //GimbalSubsystem* gimbal;

        

        // Access gimbal yaw encoder value without causing circular dependency
        std::function<float()> getYawEncoderValue_func;
};
