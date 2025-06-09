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
        if (drivers->refSerial.getRefSerialReceivingData() &&
            drivers->refSerial.getRobotData().rfidStatus & (RefSerialData::Rx::RFIDActivationStatus::RESTORATION_ZONE | RefSerialData::Rx::RFIDActivationStatus::EXCHANGE_ZONE)) {
            // not sure if it is the restoration or exchange zone that is what we use in 3v3
            shotsAtLastReload = index->getTotalNumBallsShot();
        }

        // update arc
        arc.endAngle = static_cast<uint16_t>(std::lerp(START_ANGLE, END_ANGLE, (FILLED_NUM_SHOTS - index->getTotalNumBallsShot() + shotsAtLastReload)/FILLED_NUM_SHOTS));
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

    Arc arc{UISubsystem::Color::GREEN, UISubsystem::HALF_SCREEN_WIDTH, UISubsystem::HALF_SCREEN_HEIGHT, SIZE, SIZE, START_ANGLE, END_ANGLE, THICKNESS};
};