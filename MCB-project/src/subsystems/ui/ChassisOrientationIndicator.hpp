#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class ChassisOrientationIndicator : public GraphicsContainer {
public:
    ChassisOrientationIndicator() {
        addGraphicsObject(&left);
        // addGraphicsObject(&right);
    }

    void update() final {}

private:
    static constexpr uint16_t THICKNESS = 2;       // pixels
    static constexpr uint16_t INNER_SIZE = 120;    // Used if the arcs are supposed to be inside the barrel heat circle, pixels
    static constexpr uint16_t INNER_ARC_LEN = 40;  // Used if the arcs are supposed to be inside the barrel heat circle, degrees
    static constexpr uint16_t OUTER_SIZE = 180;    // Used if the arcs are supposed to be outside the barrel heat circle, pixels
    static constexpr uint16_t OUTER_ARC_LEN = 30;  // Used if the arcs are supposed to be outside the barrel heat circle, degrees

    // Arc(RefSerialData::Tx::GraphicColor color, uint16_t cx, uint16_t cy, uint16_t width, uint16_t height, uint16_t startAngle, uint16_t endAngle, uint16_t thickness)
    Arc left{RefSerialData::Tx::GraphicColor::CYAN, UISubsystem::HALF_SCREEN_WIDTH, UISubsystem::HALF_SCREEN_HEIGHT, INNER_SIZE, INNER_SIZE, 0, INNER_ARC_LEN, THICKNESS};
    // Arc right{RefSerialData::Tx::GraphicColor::CYAN, static_cast<uint16_t>(HALF_WIDTH + BOTTOM_OFFSET), 0, static_cast<uint16_t>(HALF_WIDTH + TOP_OFFSET), HEIGHT, THICKNESS};
};