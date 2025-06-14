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
        // check if reloaded
        tap::communication::serial::RefSerial::Rx::RobotData robotData = drivers->refSerial.getRobotData();
        if (drivers->refSerial.getRefSerialReceivingData()) {
            if (robotData.rfidStatus.any(RefSerialData::Rx::RFIDActivationStatus::RESTORATION_ZONE | RefSerialData::Rx::RFIDActivationStatus::EXCHANGE_ZONE)) {
                // not sure if it is the restoration or exchange zone that is what we use in 3v3
                shotsAtLastReload = index->getTotalNumBallsShot();
            }

            float shotsShot = index->getTotalNumBallsShot() + shotsAtLastReload;
            // shots can be negative (ref system isn't able to cut off power in time to prevent shots), taproot should change the uint to int
            int16_t shotsBought = robotData.turret.bulletsRemaining17;
            if (shotsBought == 0) 
                shotsBought = robotData.turret.bulletsRemaining42;

            if (shotsBought < 0) {
                // shotsShot is lower, you have shot more than you have bought
                // shotsBought shots is higher
                arc.setLower((FILLED_NUM_SHOTS - shotsShot)/FILLED_NUM_SHOTS);
                arc.setHigher((FILLED_NUM_SHOTS - shotsShot - shotsBought)/FILLED_NUM_SHOTS);
                arc.color = UISubsystem::Color::GREEN;
            } else {
                // shotsShot is higher, haven't shot more than you have bought
                // shotsBought shots is lower
                arc.setHigher((FILLED_NUM_SHOTS - shotsShot)/FILLED_NUM_SHOTS);
                arc.setLower((FILLED_NUM_SHOTS - shotsShot - shotsBought)/FILLED_NUM_SHOTS);
                arc.color = UISubsystem::Color::PINK;
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
    float shotsAtLastReload = 0;                  // when we enter the reload zone, assume we fill completely

    LargeCenteredArc arc{false, 0};  // arc on the right size in first lane
};