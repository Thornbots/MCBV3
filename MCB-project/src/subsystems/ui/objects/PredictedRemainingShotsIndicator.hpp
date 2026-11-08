#pragma once

#include "subsystems/indexer/IndexerSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/AtomicGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class PredictedRemainingShotsIndicator : public GraphicsContainer {
public:
    PredictedRemainingShotsIndicator(src::Drivers* drivers, IndexerSubsystem* index) : drivers(drivers), index(index) { addGraphicsObject(&arc); }

    void update() {

        // shots can be negative (ref system isn't able to cut off power in time to prevent shots), taproot should change the uint to int
        #if defined(HERO)
        int16_t shotsBought = drivers->refWrapper.getBulletsRemaining42();
        #else
        // check if reloading: assume that the driver opens the hopper to reload
        // checking the rfid is unreliable
        if (drivers->remote.keyPressed(Remote::Key::Z)) {
            arc.color = UISubsystem::Color::WHITE;
            shotsAtLastReload = index->getTotalNumBallsShot();
        }
        int16_t shotsBought = drivers->refWrapper.getBulletsRemaining17();
        #endif

        float shotsShot = index->getTotalNumBallsShot() - shotsAtLastReload;


        float maxShots = FILLED_NUM_SHOTS;
        if(drivers->refWrapper.is1v1()){
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

    }

private:
    src::Drivers* drivers;
    IndexerSubsystem* index;

    #if defined(HERO)
    static constexpr int FILLED_NUM_SHOTS = 74; 
    #else
    static constexpr int FILLED_NUM_SHOTS = 729; 
    #endif

    static constexpr int MAX_SHOTS_1v1 = 200;     // if in 1v1, then you are a standard, and you can shoot only 200
    float shotsAtLastReload = 0;                  // when we enter the reload zone, assume we fill completely

    LargeCenteredArc arc{false, 0};  // arc on the right side in first lane
};