#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

// looks like
//  -----------------------------------------------------
//  | ----------------------------                      |
//  | |                          |                      | 1234
//  | ----------------------------                      |
//  -----------------------------------------------------
// at the bottom of the screen, the number is the current charge, in joules
class SupercapChargeIndicator : public GraphicsContainer {
public:
    SupercapChargeIndicator() {
        outside.color = getCurrentColor();
        inside.color = getCurrentColor();
        addGraphicsObject(&outside);
        addGraphicsObject(&inside);
    }

    void update() { inside.width = getInsideWidth(); }

private:
    static constexpr int MIN_CHARGE = 20;    // joules
    static constexpr int MAX_CHARGE = 1600;  // joules

    static constexpr uint16_t INSIDE_WIDTH = 500;      // width of the inside bar when full, pixels
    static constexpr uint16_t INSIDE_HEIGHT = 50;      // height of the inside bar, pixels
    static constexpr uint16_t HEIGHT_OFF_BOTTOM = 80;  // outside bar to the bottom of the screen, pixels
    static constexpr uint16_t PADDING = 20;            // how much room between the inside and outside bar, pixels
    static constexpr uint16_t THICKNESS = 2;           // pixels

    int getCurrentCharge() {
        // needs to get from actual supercaps at some point
        return 1234;
    }

    int getInsideWidth() { return INSIDE_WIDTH * (getCurrentCharge() - MIN_CHARGE) / (MAX_CHARGE - MIN_CHARGE); }

    RefSerialData::Tx::GraphicColor getCurrentColor() {
        // depends on if keyboard mouse or controller, could also be another color if controller disconnected
        return RefSerialData::Tx::GraphicColor::ORANGE;
    }

    UnfilledRectangle outside{
        RefSerialData::Tx::GraphicColor::CYAN,
        static_cast<uint16_t>(UISubsystem::HALF_SCREEN_WIDTH - INSIDE_WIDTH / 2 - PADDING),
        HEIGHT_OFF_BOTTOM,
        INSIDE_WIDTH + 2 * PADDING,
        INSIDE_HEIGHT + 2 * PADDING,
        THICKNESS};
    UnfilledRectangle
        inside{RefSerialData::Tx::GraphicColor::CYAN, static_cast<uint16_t>(UISubsystem::HALF_SCREEN_WIDTH - INSIDE_WIDTH / 2), HEIGHT_OFF_BOTTOM + PADDING, INSIDE_WIDTH, INSIDE_HEIGHT, THICKNESS};
};