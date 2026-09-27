#pragma once

#include "tap/communication/serial/ref_serial_data.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"


namespace subsystems
{

class HitTracker 
{
    public:
        HitTracker(tap::Drivers* drivers) : drivers(drivers), gimbal(nullptr)
        {
            // 
        }


        // Function to append a hit if the code structure already identifies a strike
        void addHit()
        {
            // Check if gimbal pointer is defined
            if (!gimbal)
            {return;}
            

            float encoder = gimbal->getYawEncoderValue() * 180 / PI;
            float imu = drivers->bmi088.getYaw();

            // damagedArmorId==0 is forward, add 0*90 degrees
            // 1 is left, add 1*90 degrees
            // 2 is back, add 2*90 degrees
            // 3 is right, add 3*90 degrees
            // 4 is top, don't care because we don't have panels on top (yet?)
            
            hitOrientation = -encoder + imu + 90 * ((uint16_t)drivers->refSerial.getRobotData().damagedArmorId);
        }
        

        // Detect if a hit occurs -- appends the hit orientation
        void update() 
        {
            // check for a new hit
            if (drivers->refSerial.getRefSerialReceivingData() && gimbal) {
                const RefSerialData::Rx::RobotData &robotData = drivers->refSerial.getRobotData();
                if (previousHp > robotData.currentHp && (robotData.damageType == RefSerialData::Rx::DamageType::ARMOR_DAMAGE || robotData.damageType == RefSerialData::Rx::DamageType::COLLISION)) {
                    // took some sort of damage and we think we took panel damage
                    addHit();
                    
                }

                previousHp = robotData.currentHp;
            }
        }


        /*
        bool IsCurrentlyHit()
        {
            // check for a new hit
            if (drivers->refSerial.getRefSerialReceivingData()) {
                RefSerialData::Rx::RobotData robotData = drivers->refSerial.getRobotData();
                if (previousHp > robotData.currentHp && (robotData.damageType == RefSerialData::Rx::DamageType::ARMOR_DAMAGE || robotData.damageType == RefSerialData::Rx::DamageType::COLLISION)) {
                    // took some sort of damage and we think we took panel damage
                    previousHp = robotData.currentHp;
                    return true;
                    
                }

                previousHp = robotData.currentHp;
            }
        }
        */


        // Returns Current Hit Orientation in degrees
        float getCurrentHit()
        {
            return hitOrientation;
        }

        
        // Return angle in radians for the sentry to turn to
        float getAngleToTurnForSentry() {
            if(hitOrientation>180) hitOrientation-=360;
            return hitOrientation * PI / 180;
        }
    
    //static constexpr float PLACEHOLDER_ANGLE = 123;  // a special value for telling jetson that you weren't hit



    private:
        tap::Drivers* drivers;
        GimbalSubsystem* gimbal;

        float hitOrientation;
        uint16_t previousHp;
};

}