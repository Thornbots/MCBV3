#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/AtomicGraphicsObjects.hpp" 

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
    ChassisOrientationIndicator(bool showPlsSpin, tap::Drivers* drivers, GimbalSubsystem* gimbal, DrivetrainSubsystem* drivetrain) : showPlsSpin(showPlsSpin), drivers(drivers), gimbal(gimbal), drivetrain(drivetrain) {
        addGraphicsObject(&front);
        addGraphicsObject(&side);
        
        if(showPlsSpin)
            addGraphicsObject(&plsSpin);
    }

    void update() {
        uint16_t heading = static_cast<uint16_t>(gimbal->getYawEncoderValue() * 180 / PI + YAW_OFFSET);
        // if the gimbal compared to the drivetrain (from the encoder) is facing forward, heading would be 0, if facing right, heading would be 90

        //front arc is convex
        front.startAngle = heading - INNER_ARC_LEN / 2;
        UISubsystem::fixAngle(&front.startAngle);
        front.endAngle = front.startAngle + INNER_ARC_LEN;

        //side arc is concave, so angle is flipped
        side.setHidden(!drivetrain->isPeeking);
        side.startAngle = (drivetrain->isPeekingLeft ? 90 : 270) + heading - INNER_ARC_LEN / 2;
        UISubsystem::fixAngle(&side.startAngle);
        side.endAngle = side.startAngle + INNER_ARC_LEN;

        // and xy location isn't the center
        float angleRadians = (drivetrain->isPeekingLeft ? PI/2 : 3*PI/2) + gimbal->getYawEncoderValue();
        side.cx = front.cx - 2*side.width*sin(angleRadians);
        side.cy = front.cy - 2*side.width*cos(angleRadians);

        //set side color to pink if on red team, cyan if on blue team
        if (drivers->refSerial.getRefSerialReceivingData()) {
            side.color = drivers->refSerial.isBlueTeam(drivers->refSerial.getRobotData().robotId) ? UISubsystem::Color::CYAN : UISubsystem::Color::PINK;
        }

        if(showPlsSpin)
            plsSpin.setHidden(!drivetrain->isPeeking && !drivetrain->isBeyblading);
    }


    
    static constexpr float YAW_OFFSET = 2*360;     // degrees, 0 from the yaw might not be top on the screen, also needs to make sure it is positive because we are using uints

private:
    bool showPlsSpin;
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

    static constexpr uint16_t TEXT_HEIGHT = 60;
    static constexpr uint16_t TEXT_Y = 830;
    static constexpr uint16_t TEXT_WIDTH = 400; //only for rectangle pls spin
    static constexpr uint16_t TEXT_THICKNESS = 5;
    // StringGraphic plsSpin{UISubsystem::Color::CYAN, "Pls spin", UISubsystem::HALF_SCREEN_WIDTH, TEXT_Y, TEXT_HEIGHT, TEXT_THICKNESS};
    UnfilledRectangle plsSpin{UISubsystem::Color::PINK, UISubsystem::HALF_SCREEN_WIDTH - TEXT_WIDTH/2, TEXT_Y, TEXT_WIDTH, TEXT_HEIGHT, TEXT_THICKNESS};

};