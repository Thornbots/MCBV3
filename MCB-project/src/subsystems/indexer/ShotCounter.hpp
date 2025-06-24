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
    using BarrelType = RefSerialData::Rx::MechanismID; 
    

    ShotCounter(src::Drivers* drivers, BarrelType barrel, tap::motor::DjiMotor* index) : drivers(drivers), barrel(barrel), index(index)  {
        //in case index doesn't start at 0
        initialPosition = index->getPositionUnwrapped();
        previousPosition = initialPosition;
        recentPosition = initialPosition;
        timeUntilNoHeat.stop();
    }

    float getAllowableIndexRate(float desiredBallsPerSecond){
        getAllowableIndexRateOld(desiredBallsPerSecond);
    }

    // Old overheating prevention
    float getAllowableIndexRateOld(float desiredBallsPerSecond) {
        tap::communication::serial::RefSerial::Rx::TurretData turretData = drivers->refSerial.getRobotData().turret;
        if (enabled && drivers->refSerial.getRefSerialReceivingData() && 
           (getHeatPerBall() * desiredBallsPerSecond - turretData.coolingRate) * LATENCY > (turretData.heatLimit - getCurrentRefHeat(&turretData))) {
            return turretData.coolingRate / getHeatPerBall();
        }
        
        return desiredBallsPerSecond;
    }

    float getAllowableIndexRateNew(float desiredBallsPerSecond) {
        //change the timer based on index encoder
        timeUntilNoHeat.restart(timeUntilNoHeat.timeRemaining() + (getNumBallsShotByReference(previousPosition)) * 1000);
        previousPosition = index->getPositionUnwrapped();

        //if can't do anything, don't do anything
        if(!enabled || !drivers->refSerial.getRefSerialReceivingData())
            return desiredBallsPerSecond;

        tap::communication::serial::RefSerial::Rx::TurretData turretData = drivers->refSerial.getRobotData().turret;
        float estimatedHeat = timeUntilNoHeat.timeRemaining() / 1000 * turretData.coolingRate; //ms to s, mult by (heat/s) to get heat
        
        if(estimatedHeat + getHeatPerBall() >= turretData.heatLimit) {
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
    

private:
    src::Drivers* drivers;
    BarrelType barrel;
    tap::motor::DjiMotor* index;

    bool enabled = true;

    tap::arch::MilliTimeout timeUntilNoHeat;
    
    int64_t recentPosition = 0;
    int64_t initialPosition = 0;
    int64_t previousPosition = 0;

    float getHeatPerBall() {
        return barrel==BarrelType::TURRET_42MM ? HEAT_PER_42 : HEAT_PER_17;
    }

    // getting from ref system has latency
    uint16_t getCurrentRefHeat(tap::communication::serial::RefSerial::Rx::TurretData* turretData){
        return barrel==BarrelType::TURRET_42MM ? turretData->heat42 : 
              (barrel==BarrelType::TURRET_17MM_1 ? turretData->heat17ID1 : turretData->heat17ID2);
    }

    float getNumBallsShotByReference(int64_t reference) {
        return (index->getPositionUnwrapped() - reference) / (REV_PER_BALL * 2 * PI);
    }
    

    static constexpr float HEAT_PER_17 = 10.0f;
    static constexpr float HEAT_PER_42 = 100.0f;

    static constexpr float LATENCY = 0.6f; //expected ref system latency for (old) barrel heat limiting
};

} // namespace subsystems
