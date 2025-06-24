#pragma once

#include "subsystems/indexer/IndexerSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class PredictedRemainingShotsIndicator : public GraphicsContainer {
public:
    PredictedRemainingShotsIndicator(tap::Drivers* drivers, IndexerSubsystem* index) : drivers(drivers), index(index) { addGraphicsObject(&arc); }

    void update() {
        if (drivers->refSerial.getRefSerialReceivingData()) {
            tap::communication::serial::RefSerial::Rx::RobotData robotData = drivers->refSerial.getRobotData();

            //check if reloaded
            bool isReloaded = false;
            if (robotData.rfidStatus.any(RefSerialData::Rx::RFIDActivationStatus::RESTORATION_ZONE | RefSerialData::Rx::RFIDActivationStatus::EXCHANGE_ZONE)) {
                // not sure if it is the restoration or exchange zone that is what we use in 3v3, should be one of them
                shotsAtLastReload = index->getTotalNumBallsShot();
                isReloaded = true;
            }

            float shotsShot = index->getTotalNumBallsShot() + shotsAtLastReload + 0.2; //add 0.2 for wiggleroom from unjam

            // shots can be negative (ref system isn't able to cut off power in time to prevent shots), taproot should change the uint to int
            #if defined(HERO)
            int16_t shotsBought = robotData.turret.bulletsRemaining42;
            #else
            int16_t shotsBought = robotData.turret.bulletsRemaining17;
            #endif

            float maxShots = FILLED_NUM_SHOTS;
            if(drivers->refSerial.getGameData().gameType == RefSerialData::Rx::GameType::ROBOMASTER_RMUL_1V1){
                maxShots = MAX_SHOTS_1v1;
            }

            if (shotsBought < 0) {
                // shotsShot is lower, you have shot more than you have bought
                // shotsBought shots is higher
                arc.setLower((maxShots - shotsShot)/maxShots);
                arc.setHigher((maxShots - shotsShot - shotsBought)/maxShots);
                arc.color = UISubsystem::Color::PINK;
            } else {
                // shotsShot is higher, haven't shot more than you have bought
                // shotsBought shots is lower
                arc.setHigher((maxShots - shotsShot)/maxShots);
                arc.setLower((maxShots - shotsShot - shotsBought)/maxShots);
                arc.color = UISubsystem::Color::GREEN;
            }

            if(isReloaded){
                arc.color = UISubsystem::Color::WHITE;
            }
        }

        // update arc
        // (FILLED_NUM_SHOTS - index->getTotalNumBallsShot() + shotsAtLastReload)/FILLED_NUM_SHOTS)
    }

private:
    tap::Drivers* drivers;
    IndexerSubsystem* index;

    #if defined(HERO)
    static constexpr int FILLED_NUM_SHOTS = 80; 
    #else
    static constexpr int FILLED_NUM_SHOTS = 500; 
    #endif

    static constexpr int MAX_SHOTS_1v1 = 200;     // if in 1v1, then you are a standard, and you can shoot only 200
    float shotsAtLastReload = 0;                  // when we enter the reload zone, assume we fill completely

    LargeCenteredArc arc{false, 0};  // arc on the right size in first lane
};