#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp" 

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"

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
    ChassisOrientationIndicator(tap::Drivers* drivers, GimbalSubsystem* gimbal, DrivetrainSubsystem* drivetrain) : drivers(drivers), gimbal(gimbal), drivetrain(drivetrain) {
        addGraphicsObject(&front);
        addGraphicsObject(&side);
    }

    void update() {
        uint16_t heading = static_cast<uint16_t>(gimbal->getYawEncoderValue() * YAW_MULT + YAW_OFFSET);
        // if the gimbal compared to the drivetrain (from the encoder) is facing forward, heading would be 0, if facing right, heading would be 90

        //front arc is convex
        front.startAngle = heading - INNER_ARC_LEN / 2;
        fixAngle(&front.startAngle);
        front.endAngle = front.startAngle + INNER_ARC_LEN;

        //side arc is concave, so angle is flipped
        side.setHidden(!drivetrain->isPeeking);
        side.startAngle = (drivetrain->isPeekingLeft ? 90 : 270) + heading - INNER_ARC_LEN / 2;
        fixAngle(&side.startAngle);
        side.endAngle = side.startAngle + INNER_ARC_LEN;

        // and xy location isn't the center
        float angleRadians = (drivetrain->isPeekingLeft ? PI/2 : 3*PI/2) + gimbal->getYawEncoderValue();
        side.cx = front.cx - 2*side.width*sin(angleRadians);
        side.cy = front.cy - 2*side.width*cos(angleRadians);

        //set side color to pink if on red team, cyan if on blue team
        if (drivers->refSerial.getRefSerialReceivingData()) {
            side.color = drivers->refSerial.isBlueTeam(drivers->refSerial.getRobotData().robotId) ? UISubsystem::Color::CYAN : UISubsystem::Color::PINK;
        }
    }


    static void fixAngle(uint16_t* a) {
        *a %= 360;  // set a to the remainder after dividing by 360, so if it was 361 it would now be 1
    }

    static constexpr float YAW_MULT = 180 / PI;  // turns radians from gimbal's getYawEncoderValue into degrees, might need to be negative
    static constexpr float YAW_OFFSET = 0;     // degrees, 0 from the yaw might not be top on the screen, also needs to make sure it is positive because we are using uints
    //YAW_OFFSET might not be necessary, need to test it (ChassisOrientationIndicator and HitRing would be affected)

private:
    tap::Drivers* drivers;
    GimbalSubsystem* gimbal;
    DrivetrainSubsystem* drivetrain;

    static constexpr uint16_t THICKNESS = 2;       // pixels
    static constexpr uint16_t INNER_SIZE = 120;    // Used if the arcs are supposed to be inside the barrel heat circle, pixels
    static constexpr uint16_t INNER_ARC_LEN = 40;  // Used if the arcs are supposed to be inside the barrel heat circle, degrees
    static constexpr uint16_t OUTER_SIZE = 180;    // Used if the arcs are supposed to be outside the barrel heat circle, pixels
    static constexpr uint16_t OUTER_ARC_LEN = 30;  // Used if the arcs are supposed to be outside the barrel heat circle, degrees

    Arc front{UISubsystem::Color::RED_AND_BLUE, UISubsystem::HALF_SCREEN_WIDTH, UISubsystem::HALF_SCREEN_HEIGHT, INNER_SIZE, INNER_SIZE, 0, 90, THICKNESS};
    Arc side{UISubsystem::Color::RED_AND_BLUE, UISubsystem::HALF_SCREEN_WIDTH, UISubsystem::HALF_SCREEN_HEIGHT, INNER_SIZE, INNER_SIZE, 0, 90, THICKNESS};
};