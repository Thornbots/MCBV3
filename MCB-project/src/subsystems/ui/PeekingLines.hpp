#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"
#include "util/ui/Projections.hpp"
#include "util/Vector3d.hpp"

using namespace subsystems;

class PeekingLines : public GraphicsContainer {
public:
    PeekingLines(GimbalSubsystem* gimbal) : gimbal(gimbal) {
        addGraphicsObject(&left);
        addGraphicsObject(&right);
    }

    void update() {
        Vector3d vs{MAGNITUDE, 0, 0};
        Vector3d temp;
        Vector3d temp2;
        Vector2d temp3;
        uint16_t xPositions[4];
        for(int i=0; i<4; i++){
            //currently the point is to the right, needs rotated forward
            temp = vs.rotateYaw(ANGLES[i] + gimbal->getYawEncoderValue()); 
            //now we are still in robot space, need to get to pivot
            temp2 = Projections::robotSpaceToPivotSpace(temp);
            //now in pivot, since we don't care about y we won't rotate by pitch
            temp = Projections::pivotSpaceToVtmSpace(temp2);
            //now get to screen space
            temp3 = Projections::vtmSpaceToScreenSpace(temp);
            xPositions[i] =  static_cast<uint16_t>(std::clamp(temp3.getX(), 0.0f, static_cast<float>(UISubsystem::SCREEN_WIDTH)));
        }
        left.x = std::min(xPositions[0], xPositions[1]);
        left.width = xPositions[0]>xPositions[1] ? xPositions[0]-xPositions[1] : xPositions[1]-xPositions[0];
        right.x = std::min(xPositions[3], xPositions[4]);
        right.width = xPositions[3]>xPositions[4] ? xPositions[3]-xPositions[4] : xPositions[4]-xPositions[3];
    }

private:
    GimbalSubsystem* gimbal;

    //these numbers need calculated, would be robot dependent but the infantry is the only robot
    //to draw peeking lines, these are for knowing what sector the enemy has to be in for them to
    //not be able to shoot at us if we are hiding our front panel behind a wall. ANGLE1 determines 
    //the inner edge of the rectangle, the one used to align with a wall. ANGLE2 determines the 
    //outer edge of the rectangle, the one used to mark how far away from the wall the enemy can be
    //and still not be able to hit us. As a driver, the goal while peeking is to keep the enemy inside
    //of these bounds.
    static constexpr float ANGLE1 = 74*PI/180; //zero would be directly to the right/left, 90 would be directly forward
    static constexpr float ANGLE2 = 60*PI/180;
    static constexpr float ANGLES[4] = {ANGLE1, ANGLE2, PI-ANGLE1, PI-ANGLE2};
    static constexpr float MAGNITUDE = 0.3; //need a magnitude to make a 3d point to project

    static constexpr uint16_t BOTTOM_OFFSET = 480; //y distance from the end of the line to the bottom of the screen
    static constexpr uint16_t TOP_OFFSET = 320; //y distance from the top of the line to the top of the screen
    static constexpr uint16_t THICKNESS = 2; //pixels

    Line line{UISubsystem::Color::CYAN, 0, 0, 0, 0, THICKNESS};

    UnfilledRectangle left{UISubsystem::Color::CYAN, 0, 0, 0, 0, THICKNESS};
    UnfilledRectangle right{UISubsystem::Color::CYAN, 0, 0, 0, 0, THICKNESS};

    
};