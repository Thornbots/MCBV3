#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class PeekingLines : public GraphicsContainer {
public:
    PeekingLines() {
        // addGraphicsObject(&left);
        // addGraphicsObject(&right);
    }

    void update() final {
        
    }

private:
    
    static constexpr uint16_t DISTANCE_FROM_CENTER = 80; //0 would make the peeking lines in the center of the screen
    static constexpr uint16_t BOTTOM_OFFSET = 480; //distance from the end of the line to the bottom of the screen
    static constexpr uint16_t TOP_OFFSET = 320; //distance from the top of the line to the top of the screen
    static constexpr uint16_t THICKNESS = 2; //pixels

    Line left{RefSerialData::Tx::GraphicColor::CYAN, static_cast<uint16_t>(UISubsystem::HALF_SCREEN_WIDTH - DISTANCE_FROM_CENTER), BOTTOM_OFFSET, static_cast<uint16_t>(UISubsystem::HALF_SCREEN_WIDTH - DISTANCE_FROM_CENTER), static_cast<uint16_t>(UISubsystem::SCREEN_HEIGHT - TOP_OFFSET), THICKNESS};
    Line right{RefSerialData::Tx::GraphicColor::CYAN, static_cast<uint16_t>(UISubsystem::HALF_SCREEN_WIDTH + DISTANCE_FROM_CENTER), BOTTOM_OFFSET, static_cast<uint16_t>(UISubsystem::HALF_SCREEN_WIDTH + DISTANCE_FROM_CENTER), static_cast<uint16_t>(UISubsystem::SCREEN_HEIGHT - TOP_OFFSET), THICKNESS};
};