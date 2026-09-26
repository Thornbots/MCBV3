#pragma once

#include "subsystems/gimbal/GimbalSubsystem.hpp"


namespace subsystems
{

class HitTrackerSubsystem : public tap::control::Subsystem
{
    public:
        HitTrackerSubsystem(tap::Drivers* drivers, GimbalSubsystem* gimbal_input) : tap::control::Subsystem(drivers), drivers(drivers), refSerialTransmitter(drivers)
        {
            //drivers = drivers_input;
            gimbal = gimbal_input;
        }


        // Function to append a hit if the code structure already identifies a strike
        void AddHit(RefSerialData::Rx::RobotData *robotData, int index = -1)
        {
            if (index == -1)
            {index = nextIndex;}

            float encoder = gimbal->getYawEncoderValue() * 180 / PI;
            float imu = drivers->bmi088.getYaw();

            // damagedArmorId==0 is forward, add 0*90 degrees
            // 1 is left, add 1*90 degrees
            // 2 is back, add 2*90 degrees
            // 3 is right, add 3*90 degrees
            // 4 is top, don't care because we don't have panels on top (yet?)
            
            hitOrientations[nextIndex] = -encoder + imu + 90 * ((uint16_t)robotData->damagedArmorId);
            
            // get the next index 
            nextIndex++;
            if (nextIndex == NUM_HISTORY) nextIndex = 0;  // cycle back around and overwrite if we get hit really often
        }
        

        // Detect if a hit occurs -- appends the hit orientation
        void Update() 
        {
            // check for a new hit
            if (drivers->refSerial.getRefSerialReceivingData()) {
                RefSerialData::Rx::RobotData robotData = drivers->refSerial.getRobotData();
                if (previousHp > robotData.currentHp && (robotData.damageType == RefSerialData::Rx::DamageType::ARMOR_DAMAGE || robotData.damageType == RefSerialData::Rx::DamageType::COLLISION)) {
                    // took some sort of damage and we think we took panel damage
                    AddHit(&robotData);
                    
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


        // Returns Current Hit Orientation
        float GetCurrentHit(int index = -1)
        {
            // Allow previous angle to be indexed
            if(index == -1)
            {index = nextIndex;}

            return hitOrientations[nextIndex];
        }

        
        float getAngleToTurnForSentry() {
            if(expirationTimeouts[0].isStopped())
                return PLACEHOLDER_ANGLE;

            expirationTimeouts[0].stop();
            float inDegrees = hitOrientations[nextIndex]; //rings[0].startAngle + ARC_LEN / 2;
            if(inDegrees>180) inDegrees-=360;
            return inDegrees * PI / 180;
    }
    
    static constexpr float PLACEHOLDER_ANGLE = 123;  // a special value for telling jetson that you weren't hit



    private:
        tap::Drivers* drivers;
        GimbalSubsystem* gimbal;
        RefSerialTransmitter refSerialTransmitter;

        // Iterate through the array every time a hit occurs
        int nextIndex = 0;

        // Hit Detection Storage
        // --------------------------------
        // Number of hits to remember
        #if defined(SENTRY)
        static constexpr int NUM_HISTORY = 1;  // keep track of 1 to send to jetson, when we send it skip the timer and expire it
        #else
        static constexpr int NUM_HISTORY = 3;  // how many shots to keep track of
        #endif

        float hitOrientations[NUM_HISTORY];

        
        tap::arch::MilliTimeout expirationTimeouts[NUM_HISTORY];  // for knowing how old a hit is, stopped if not hit recently
        uint16_t previousHp;

        
        
    

};

}