#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp" 

#include "subsystems/gimbal/GimbalSubsystem.hpp"

using namespace subsystems;

// looks like
//    __
//   /
//
//      __/
// at the center of the screen, the arcs represent the left and right inner 
// panels if there are 2 arcs, if there are 4 then they are all four panels
class ChassisOrientationIndicator : public GraphicsContainer {
public:
    ChassisOrientationIndicator(GimbalSubsystem* gimbal) : gimbal(gimbal) {
        addGraphicsObject(&left);
        addGraphicsObject(&right);
    }

    void update() {
        uint16_t heading = static_cast<uint16_t>(gimbal->getYawEncoderValue() * YAW_MULT + YAW_OFFSET);
        // if the gimbal compared to the drivetrain (from the encoder) is facing forward, heading would be 0, if facing right, heading would be 90

        left.startAngle = 270 + heading - INNER_ARC_LEN / 2;
        fixAngle(&left.startAngle);
        left.endAngle = left.startAngle + INNER_ARC_LEN;

        right.startAngle = 90 + heading - INNER_ARC_LEN / 2;
        fixAngle(&right.startAngle);
        right.endAngle = right.startAngle + INNER_ARC_LEN;
    }


    void fixAngle(uint16_t* a) {
        *a %= 360;  // set a to the remainder after dividing by 360, so if it was 361 it would now be 1
    }

private:
    GimbalSubsystem* gimbal;

    static constexpr uint16_t THICKNESS = 2;       // pixels
    static constexpr uint16_t INNER_SIZE = 120;    // Used if the arcs are supposed to be inside the barrel heat circle, pixels
    static constexpr uint16_t INNER_ARC_LEN = 40;  // Used if the arcs are supposed to be inside the barrel heat circle, degrees
    static constexpr uint16_t OUTER_SIZE = 180;    // Used if the arcs are supposed to be outside the barrel heat circle, pixels
    static constexpr uint16_t OUTER_ARC_LEN = 30;  // Used if the arcs are supposed to be outside the barrel heat circle, degrees

    static constexpr float YAW_MULT = 180 / PI;  // turns radians from gimbal's getYawEncoderValue into degrees, might need to be negative
    static constexpr float YAW_OFFSET = 360;     // degrees, 0 from the yaw might not be top on the screen, also needs to make sure it is positive because we are using uints

    Arc left{UISubsystem::Color::RED_AND_BLUE, UISubsystem::HALF_SCREEN_WIDTH, UISubsystem::HALF_SCREEN_HEIGHT, INNER_SIZE, INNER_SIZE, 0, 90, THICKNESS};
    Arc right{UISubsystem::Color::RED_AND_BLUE, UISubsystem::HALF_SCREEN_WIDTH, UISubsystem::HALF_SCREEN_HEIGHT, INNER_SIZE, INNER_SIZE, 0, 90, THICKNESS};
};