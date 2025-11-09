#pragma once
#include "IndexerSubsystemConstants.hpp"

#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

#include "drivers.hpp"

namespace subsystems{

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
        //in case index doesn't start at 0
        initialPosition = index->getPositionUnwrapped();
        recentPosition = initialPosition;
        timeUntilNoHeat.stop();
    }

    float getAllowableIndexRate(float desiredBallsPerSecond){
        tap::communication::serial::RefSerial::Rx::TurretData turretData = drivers->refSerial.getRobotData().turret;
        if (enabled && drivers->refSerial.getRefSerialReceivingData() && 
           (getHeatPerBall() * desiredBallsPerSecond - turretData.coolingRate) * LATENCY > (turretData.heatLimit - getCurrentRefHeat(&turretData))) {
            return turretData.coolingRate / getHeatPerBall();
        }
        
        return desiredBallsPerSecond;
    }

    float getTotalNumBallsShot() {
        return getNumBallsShotByReference(initialPosition);
    }

    float getRecentNumBallsShot() {
        return getNumBallsShotByReference(recentPosition);
    }

    void resetRecentBallsCounter() {
        recentPosition = index->getPositionUnwrapped();
    }

    void enable() {
        enabled = true;
    }

    void disable() {
        enabled = false;
    }
    
    // for shooting one ball, how far does the index need to move
    int64_t getPositionIncrement() {
        // inverse of getNumBallsShotByReference, but you don't need to subtract a reference
        return (REV_PER_BALL * 2 * PI);
    }
    

private:
    src::Drivers* drivers;
    BarrelType barrel;
    tap::motor::DjiMotor* index;

    bool enabled = true;

    tap::arch::MilliTimeout timeUntilNoHeat;
    
    int64_t recentPosition = 0;
    int64_t initialPosition = 0;

    float getHeatPerBall() {
        return barrel==BarrelType::TURRET_42MM ? HEAT_PER_42 : HEAT_PER_17;
    }

    // getting from ref system has latency
    uint16_t getCurrentRefHeat(tap::communication::serial::RefSerial::Rx::TurretData* turretData){
        return barrel==BarrelType::TURRET_42MM ? turretData->heat42 : 
              (barrel==BarrelType::TURRET_17MM_EITHER ? std::max(turretData->heat17ID1, turretData->heat17ID2) :
              (barrel==BarrelType::TURRET_17MM_1 ? turretData->heat17ID1 : turretData->heat17ID2));
    }

    float getNumBallsShotByReference(int64_t reference) {
        return (index->getPositionUnwrapped() - reference) / (REV_PER_BALL * 2 * PI);
    }
    

    static constexpr float HEAT_PER_17 = 10.0f;
    static constexpr float HEAT_PER_42 = 100.0f;

    #if defined(HERO)
    static constexpr float LATENCY = 0; //allow hero to shoot one shot at 20hz, up to driver to not overheat
    #else
    static constexpr float LATENCY = 0.6f; //expected ref system latency for (old) barrel heat limiting
    #endif
};

} // namespace subsystems
