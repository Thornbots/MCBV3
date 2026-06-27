#pragma once
#include "IndexerSubsystemConstants.hpp"

#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

#include "drivers.hpp"

namespace subsystems{
    
// keeps track of heat

using namespace tap::communication::serial;

class ShotCounter
{

public:
    enum BarrelType
    {
        TURRET_17MM_EITHER = 0,  ///< 17mm barrel ID 1 or 2, uses the max, so for standard and don't have to worry if smm is on id 1 or not
        TURRET_17MM_1 = 1,  ///< 17mm barrel ID 1
        TURRET_17MM_2 = 2,  ///< 17mm barrel ID 2
        TURRET_42MM = 3,    ///< 42mm barrel
    };


    ShotCounter(src::Drivers* drivers, BarrelType barrel, tap::motor::DjiMotor* index) : drivers(drivers), barrel(barrel), index(index)  {
        resetAll();
    }

    bool canShootAgain() {
        return estHeat+getHeatPerBall()+getHeatPerBall()/2 < drivers->refSerial.getRobotData().turret.heatLimit;
    }

    uint32_t getTimesIncremented() {
        return timesIncremented;
    }

    void resetAll() {
        //in case index doesn't start at 0
        timesIncremented=0;

        prevMillis = tap::arch::clock::getTimeMilliseconds();
        
        coolingTimer.restart(); //not sure if it starts started or not
    }

    int32_t getEstHeat(){
        return estHeat;
    }
    
    float getEstHeatRatio() {
        if(drivers->refSerial.getRefSerialReceivingData())
            return ((float) estHeat)/drivers->refSerial.getRobotData().turret.heatLimit;
        return 0;
    }
    
    
    void update() {
        if(coolingTimer.execute())
            applyCooling();
    }
    
    // applies the heat from this projectile instantly
    void incrementTargetNumBalls(){
        estHeat+=getHeatPerBall();
    }
    

private:
    src::Drivers* drivers;
    BarrelType barrel;
    tap::motor::DjiMotor* index;
    
    tap::arch::PeriodicMilliTimer coolingTimer{100}; //10hz, ref system cools at this rate

    uint32_t prevMillis = 0; //for decrementing heat
    int32_t estHeat = 0;
    
    uint32_t timesIncremented = 0;

    float getHeatPerBall() {
        return barrel==BarrelType::TURRET_42MM ? HEAT_PER_42 : HEAT_PER_17;
    }

    void applyCooling() {
        // delta time[ms] * cooling[heat/s] / 1000[ms to s]
        int32_t heatDiff = (tap::arch::clock::getTimeMilliseconds()-prevMillis)*(drivers->refSerial.getRobotData().turret.coolingRate)/1000;
        if(heatDiff>0){ //could be 0 if not enough time has passed
            estHeat-=heatDiff;
            if(estHeat<0) estHeat=0; //can't be negative heat

            // change the time by the amount of cooling you applied
            prevMillis += heatDiff*1000/(drivers->refSerial.getRobotData().turret.coolingRate);
        }
    }

    static constexpr float HEAT_PER_17 = 10.0f;
    static constexpr float HEAT_PER_42 = 100.0f;
};

} // namespace subsystems
