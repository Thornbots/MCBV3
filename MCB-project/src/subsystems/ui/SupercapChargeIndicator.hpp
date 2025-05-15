#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

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
    SupercapChargeIndicator(DrivetrainSubsystem* drivetrain) : drivetrain(drivetrain) {
        outside.color = getCurrentColor();
        inside.color = getCurrentColor();
        inside.width = getInsideWidth();
        addGraphicsObject(&outside);
        addGraphicsObject(&inside);
    }

    void update() {
        outside.color = getCurrentColor();
        inside.color = getCurrentColor();
        inside.width = getInsideWidth();
    }

private:
    DrivetrainSubsystem* drivetrain;

    static constexpr uint16_t INSIDE_WIDTH = 500;      // width of the inside bar when full, pixels
    static constexpr uint16_t INSIDE_HEIGHT = 50;      // height of the inside bar, pixels
    static constexpr uint16_t HEIGHT_OFF_BOTTOM = 80;  // outside bar to the bottom of the screen, pixels
    static constexpr uint16_t PADDING = 20;            // how much room between the inside and outside bar, pixels
    static constexpr uint16_t THICKNESS = 2;           // pixels


    static constexpr int MIN_CHARGE = 20;    // joules
    static constexpr int MAX_CHARGE = 1600;  // joules
    int getCurrentCharge() {
        // needs to get from actual supercaps at some point
        return 1234;
    }

    int getInsideWidth() { return INSIDE_WIDTH * (getCurrentCharge() - MIN_CHARGE) / (MAX_CHARGE - MIN_CHARGE); }

    UISubsystem::Color getCurrentColor() {
        //green if in keyboard mode, pink if in controller mode, black if neither (controller is probably off)
        return drivetrain->isInKeyboardMode ? UISubsystem::Color::GREEN : drivetrain->isInControllerMode ? UISubsystem::Color::PINK : UISubsystem::Color::BLACK;
    }

    UnfilledRectangle outside{
        UISubsystem::Color::CYAN,
        UISubsystem::HALF_SCREEN_WIDTH - INSIDE_WIDTH / 2 - PADDING,
        HEIGHT_OFF_BOTTOM,
        INSIDE_WIDTH + 2 * PADDING,
        INSIDE_HEIGHT + 2 * PADDING,
        THICKNESS};
    UnfilledRectangle inside{UISubsystem::Color::CYAN, UISubsystem::HALF_SCREEN_WIDTH - INSIDE_WIDTH / 2, HEIGHT_OFF_BOTTOM + PADDING, INSIDE_WIDTH, INSIDE_HEIGHT, THICKNESS};
};