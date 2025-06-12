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
        resetRecentBallsCounter();
    }

    float getAllowableIndexRate(float desiredBallsPerSecond) {
        // Old overheating prevention
        tap::communication::serial::RefSerial::Rx::TurretData turretData = drivers->refSerial.getRobotData().turret;
        if (enabled && drivers->refSerial.getRefSerialReceivingData() && 
           (getHeatPerBall() * desiredBallsPerSecond - turretData.coolingRate) * LATENCY > (turretData.heatLimit - getCurrentHeat(&turretData))) {
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

    
    int64_t recentPosition = 0;
    int64_t initialPosition = 0;

    float getHeatPerBall() {
        return barrel==BarrelType::TURRET_42MM ? HEAT_PER_42 : HEAT_PER_17;
    }

    uint16_t getCurrentHeat(tap::communication::serial::RefSerial::Rx::TurretData* turretData){
        return barrel==BarrelType::TURRET_42MM ? turretData->heat42 : 
              (barrel==BarrelType::TURRET_17MM_1 ? turretData->heat17ID1 : turretData->heat17ID2);
    }

    float getNumBallsShotByReference(int64_t reference) {
        return (index->getPositionUnwrapped() - reference) / (REV_PER_BALL * 2 * PI);
    }
    

    static constexpr float HEAT_PER_17 = 10.0f;
    static constexpr float HEAT_PER_42 = 100.0f;

    static constexpr float LATENCY = 0.6f; //expected ref system latency for barrel heat limiting
};

} // namespace subsystems
