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

            float shotsShot = index->getTotalNumBallsShot() + shotsAtLastReload;

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

    static constexpr uint16_t THICKNESS = 5;      // pixels
    static constexpr uint16_t START_ANGLE = 227;  // degrees, lines up with the bottom of the left parenthesis thingy that is drawn by default
    static constexpr uint16_t END_ANGLE = 313;    // degrees, lines up with the top
    static constexpr uint16_t SIZE = 392;         // pixels, makes it so we are just inside the left parenthesis thingy

    static constexpr int FILLED_NUM_SHOTS = 500;  // this would be ifdefed, but only infantry draws it right now
    static constexpr int MAX_SHOTS_1v1 = 200;     // if in 1v1, then you are a standard, and you can shoot only 200
    float shotsAtLastReload = 0;                  // when we enter the reload zone, assume we fill completely

    LargeCenteredArc arc{false, 0};  // arc on the right size in first lane
};