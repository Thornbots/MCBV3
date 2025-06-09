#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

#include "subsystems/indexer/IndexerSubsystem.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class PredictedRemainingShotsIndicator : public GraphicsContainer {
public:
    PredictedRemainingShotsIndicator(IndexerSubsystem* index) : index(index) {
        addGraphicsObject(&arc);
    }

    void update() {
        arc.endAngle = static_cast<uint16_t>(std::lerp(START_ANGLE, END_ANGLE, 0.1));
    }

private:
    IndexerSubsystem* index;
    static constexpr uint16_t THICKNESS = 5;       // pixels
    
    static constexpr uint16_t START_ANGLE = 227; //degrees, lines up with the bottom of the left parenthesis thingy that is drawn by default
    static constexpr uint16_t END_ANGLE = 313;   //degrees, lines up with the top
    static constexpr uint16_t SIZE = 392;        //pixels, makes it so we are just inside the left parenthesis thingy

    Arc arc{UISubsystem::Color::GREEN, UISubsystem::HALF_SCREEN_WIDTH, UISubsystem::HALF_SCREEN_HEIGHT, SIZE, SIZE, START_ANGLE, END_ANGLE, THICKNESS};
};