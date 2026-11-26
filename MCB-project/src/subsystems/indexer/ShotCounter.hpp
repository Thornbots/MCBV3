#pragma once
#include "LauncherHeat.hpp"

#include "tap/communication/serial/ref_serial_data.hpp"
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
        return heat.canCharge(drivers->refSerial.getRobotData().turret.heatLimit,
                              getHeatPerBall() * HEAT_PER_BALL_MULT);
    }

    uint32_t getTimesIncremented() {
        return timesIncremented;
    }

    void resetAll() {
        //in case index doesn't start at 0
        timesIncremented=0;

        heat.reset(tap::arch::clock::getTimeMilliseconds());
    }

    int32_t getEstHeat(){
        return heat.value();
    }
    
    float getEstHeatRatio() {
        const auto limit = drivers->refSerial.getRobotData().turret.heatLimit;
        if (limit && drivers->refSerial.getRefSerialReceivingData())
            return static_cast<float>(heat.value()) / limit;
        return 0;
    }
    
    
    void update() {
        heat.cool(tap::arch::clock::getTimeMilliseconds(),
                  drivers->refSerial.getRobotData().turret.coolingRate);
    }
    
    // applies the heat from this projectile instantly
    void incrementTargetNumBalls(){
        timesIncremented++;
        heat.charge(getHeatPerBall());
    }
    

private:
    src::Drivers* drivers;
    BarrelType barrel;
    tap::motor::DjiMotor* index;
    
    LauncherHeat heat;
    uint32_t timesIncremented = 0;

    uint32_t getHeatPerBall() const {
        return barrel == BarrelType::TURRET_42MM ? kHeatPer42mm : kHeatPer17mm;
    }
    
    // heat limiting isn't perfect, this tunes how careful heat limiting is. Tunes 'shoot if there is enough room for exactly HEAT_PER_BALL_MULT projectiles'
#if defined(HERO)
    static constexpr uint32_t HEAT_PER_BALL_MULT = 1;
#else
    static constexpr uint32_t HEAT_PER_BALL_MULT = kSentryEstimatedRounds;
#endif
};

} // namespace subsystems
