#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class PeekingLines : public GraphicsContainer {
public:
    PeekingLines() {
        addGraphicsObject(&line);
    }

private:
    
    static constexpr uint16_t DISTANCE_FROM_CENTER = 100; //0 would make the peeking lines in the center of the screen
    static constexpr uint16_t BOTTOM_OFFSET = 480; //distance from the end of the line to the bottom of the screen
    static constexpr uint16_t TOP_OFFSET = 320; //distance from the top of the line to the top of the screen
    static constexpr uint16_t THICKNESS = 2; //pixels

    Line line{RefSerialData::Tx::GraphicColor::CYAN, 0, 0, 0, 0, THICKNESS};

    void setLineLeft() {
        line.x1 = static_cast<uint16_t>(UISubsystem::HALF_SCREEN_WIDTH - DISTANCE_FROM_CENTER);
        line.y1 = BOTTOM_OFFSET;
        line.x2 = line.x1;
        line.y2 = static_cast<uint16_t>(UISubsystem::SCREEN_HEIGHT - TOP_OFFSET);
    }

    void setLineRight() {
        line.x1 = static_cast<uint16_t>(UISubsystem::HALF_SCREEN_WIDTH + DISTANCE_FROM_CENTER);
        line.y1 = BOTTOM_OFFSET;
        line.x2 = line.x1;
        line.y2 = static_cast<uint16_t>(UISubsystem::SCREEN_HEIGHT - TOP_OFFSET);
    }

    void setLineNone() {
        line.x1 = 0;
        line.x2 = 0;
        line.y1 = 0;
        line.y2 = 0;
    }
};