#pragma once

#include "UISubsystem.hpp"
#include "Projections.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"
#include "util/Vector3d.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"

using namespace subsystems;

class PeekingLines : public GraphicsContainer {
public:
    PeekingLines(DrivetrainSubsystem* drivetrain, GimbalSubsystem* gimbal) : gimbal(gimbal), drivetrain(drivetrain) {
        addGraphicsObject(&left);
        addGraphicsObject(&right);
    }

    void update() {
        if(!drivetrain->isPeeking){
            left.x = 0;
            left.width = 0;
            right.x = 0;
            right.width = 0;
        } else {
            Vector3d vs{MAGNITUDE, 0, 0};
            Vector3d temp;
            Vector3d temp2;
            Vector2d temp3;
            float xPositions[4];
            for(int i=0; i<4; i++){
                //currently the point is to the right, needs rotated forward
                temp = vs.rotateYaw(ANGLES[i] + gimbal->getYawEncoderValue()); 
                //now we are still in robot space, need to get to pivot
                temp2 = Projections::robotSpaceToPivotSpace(temp);
                //now in pivot, since we don't care about y we won't rotate by pitch
                temp = Projections::pivotSpaceToVtmSpace(temp2);
                //now get to screen space
                temp3 = Projections::vtmSpaceToScreenSpace(temp);
                xPositions[i] =  std::clamp(static_cast<uint16_t>(temp3.getX()), static_cast<uint16_t>(0), (UISubsystem::SCREEN_WIDTH));
            }
            left.x = std::min(xPositions[0], xPositions[1]);
            left.width = xPositions[0]>xPositions[1] ? xPositions[0]-xPositions[1] : xPositions[1]-xPositions[0];
            if(left.width == UISubsystem::SCREEN_WIDTH) left.width = 0;
            right.x = std::min(xPositions[2], xPositions[3]);
            right.width = xPositions[2]>xPositions[3] ? xPositions[2]-xPositions[3] : xPositions[3]-xPositions[2];
            if(right.width == UISubsystem::SCREEN_WIDTH) right.width = 0;
        }
    }

private:
    GimbalSubsystem* gimbal;
    DrivetrainSubsystem* drivetrain;

    //these numbers need calculated, would be robot dependent but the infantry is the only robot
    //to draw peeking lines, these are for knowing what sector the enemy has to be in for them to
    //not be able to shoot at us if we are hiding our front panel behind a wall. ANGLE1 determines 
    //the inner edge of the rectangle, the one used to align with a wall. ANGLE2 determines the 
    //outer edge of the rectangle, the one used to mark how far away from the wall the enemy can be
    //and still not be able to hit us. As a driver, the goal while peeking is to keep the enemy inside
    //of these bounds.
    static constexpr float ANGLE1 = PI - PEEK_RIGHT_AMT; //zero would be directly to the right/left, 90 would be directly forward
    static constexpr float ANGLE2 = PI - PEEK_RIGHT_AMT - 0.16;
    static constexpr float ANGLES[4] = {ANGLE1, ANGLE2, PI-ANGLE1, PI-ANGLE2};
    static constexpr float MAGNITUDE = 0.3; //need a magnitude to make a 3d point to project

    static constexpr uint16_t BOTTOM_OFFSET = 0; //y distance from the end of the line to the bottom of the screen
    static constexpr uint16_t HEIGHT = UISubsystem::SCREEN_HEIGHT; //height of both rectangles
    static constexpr uint16_t THICKNESS = 2; //pixels

    UnfilledRectangle left{UISubsystem::Color::CYAN, 0, BOTTOM_OFFSET, 0, HEIGHT, THICKNESS};
    UnfilledRectangle right{UISubsystem::Color::PURPLISH_RED, 0, BOTTOM_OFFSET, 0, HEIGHT, THICKNESS};

    
};