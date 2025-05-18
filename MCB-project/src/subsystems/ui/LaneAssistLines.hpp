#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"
#include "util/ui/Projections.hpp"
#include "util/Vector3d.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"

using namespace subsystems;

// looks like / \ at the bottom of the screen
class LaneAssistLines : public GraphicsContainer {
public:
    LaneAssistLines(GimbalSubsystem* gimbal) : gimbal(gimbal) {
        addGraphicsObject(&left);
        addGraphicsObject(&right);
    }

    void update() {
        float pitch = gimbal->getPitchEncoderValue();
        //vs's in robot space, don't need to rotate with drivetrain, does need to rotate with pitch
        Vector3d vs[4];
        vs[0] = Vector3d{ROBOT_RADIUS, FORWARD_DISTANCE, 0};
        vs[1] = Vector3d{ROBOT_RADIUS, FORWARD_DISTANCE - ROBOT_RADIUS, 0};
        vs[2] = Vector3d{-ROBOT_RADIUS, FORWARD_DISTANCE, 0};
        vs[3] = Vector3d{-ROBOT_RADIUS, FORWARD_DISTANCE - ROBOT_RADIUS, 0};
        Vector3d temp;
        Vector3d temp2;
        Vector2d temp3;
        float xPositions[4];
        float yPositions[4];
        for(int i=0; i<4; i++){
            //currently  we are in robot space, need to get to pivot
            temp2 = Projections::robotSpaceToPivotSpace(vs[i]);
            //now in pivot, rotate by pitch
            temp = temp2.rotatePitch(pitch);
            //now that we rotated around pivot, go to vtm
            temp2 = Projections::pivotSpaceToVtmSpace(temp);
            //now get to screen space
            temp3 = Projections::vtmSpaceToScreenSpace(temp2);


            if(i%2==0){
                //0 or 2, put into x and y as is unless y is less than 0, if it is then stop this loop and not show any lines
                if(temp3.getY()<0){
                    //would mean that the entire line would be below the screen, so don't try to calculate the other point
                    this->hide(); //hide all the lines I contain
                    return;
                }
                xPositions[i] = temp3.getX();
                yPositions[i] = temp3.getY();
            } else {
                //1 or 3, x and y get extended to the bottom of the screen
                xPositions[i] = xPositions[i-1] - (yPositions[i-1])*(xPositions[i-1]-xPositions[i])/(yPositions[i-1]-yPositions[i]);
                yPositions[i] = 0;
            }
        }

        this->show(); //show all the lines I contain
        right.x1 = xPositions[0];
        right.x2 = xPositions[1];
        left.x1 = xPositions[2];
        left.x2 = xPositions[3];
        
        right.y1 = yPositions[0];
        right.y2 = yPositions[1];
        left.y1 = yPositions[2];
        left.y2 = yPositions[3];
    }


private:
#if defined(HERO)        // todo
static constexpr float ROBOT_RADIUS = 0.3; //meters
static constexpr float FORWARD_DISTANCE = 1.2; //meters, how far forward the laneassistlines extend to
#elif defined(SENTRY)    // todo
static constexpr float ROBOT_RADIUS = 0.3; //meters
static constexpr float FORWARD_DISTANCE = 1.2; //meters, how far forward the laneassistlines extend to
#elif defined(INFANTRY)  
    static constexpr float ROBOT_RADIUS = 0.3; //meters
    static constexpr float FORWARD_DISTANCE = 1.2; //meters, how far forward the laneassistlines extend to
#else                    // old infantry, todo
static constexpr float ROBOT_RADIUS = 0.3; //meters
static constexpr float FORWARD_DISTANCE = 1.2; //meters, how far forward the laneassistlines extend to
#endif

    static constexpr uint16_t THICKNESS = 2;  // pixels

    GimbalSubsystem* gimbal = nullptr;

    Line left{UISubsystem::Color::CYAN, 0, 0, 0, 0, THICKNESS};
    Line right{UISubsystem::Color::CYAN, 0, 0, 0, 0, THICKNESS};
};