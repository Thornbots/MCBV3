#pragma once

#include "subsystems/jetson/JetsonSubsystemConstants.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"
#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class Reticle : public GraphicsContainer {
public:
    Reticle(GimbalSubsystem* gimbal) : gimbal(gimbal) {
        for(int i=0;i<NUM_THINGS;i++){
            rects[i].color = COLORS[i];
            lines[i].color = COLORS[i];
            rectsContainer.addGraphicsObject(rects+i);
            linesContainer.addGraphicsObject(lines+i);
        }
        
        addGraphicsObject(&rectsContainer);
        addGraphicsObject(&linesContainer);
        // addGraphicsObject(&curvesContainer);

        updateHidden();

    }

    void update() {
        float pitch = gimbal->getPitchEncoderValue();
        Vector3d temp{0,initialShotVelocity,0};
        Vector3d initialVelo = temp.rotatePitch(-pitch);

        Vector3d initialPos{0,0,0}; //shot starts in barrel space
        temp = Projections::barrelSpaceToPivotSpace(initialPos);
        initialPos = temp.rotatePitch(-pitch);

        //initialPos and initialVelo are in pivot space, parallel to the ground so gravity is pulling in the -z direction only
        
        //NUM_THINGS applies with rectangles and lines (so far, other modes may be added)
        if(mode==ReticleDrawMode::RECTANGLES || mode==ReticleDrawMode::LINES)
            for(int i=0;i<NUM_THINGS;i++){
                float t = (DISTANCES[i] - initialPos.getY()) / initialVelo.getY(); //(distance to travel [meters]) divided by (speed to get there [meters/seconds]) gives (time to get there [seconds])
                float zFinal = initialPos.getZ() + initialVelo.getZ() * t - tap::algorithms::ACCELERATION_GRAVITY/2 * t * t; //make sure gravity is negative, the taproot constant is positive, need to subtract
                Vector3d landingSpot{initialPos.getX(), DISTANCES[i], zFinal}; //side to side doesn't change, we are defining the down range distance, and we calculated the height off the ground

                if(mode==ReticleDrawMode::RECTANGLES){
                    //need the edges because the corners will trace a trapezoid
                    Vector2d right = project(landingSpot + panelEdges[0], pitch); //gives width by right x minus left x
                    Vector2d left = project(landingSpot + panelEdges[1], pitch); //gives x of rect
                    Vector2d top = project(landingSpot + panelEdges[2], pitch); //gives height by top y minus bottom y
                    Vector2d bottom = project(landingSpot + panelEdges[3], pitch); //gives y of rect

                    rects[i].x = left.getX();
                    rects[i].width = right.getX() - left.getX();
                    rects[i].y = bottom.getY();
                    rects[i].height = top.getY() - bottom.getY();
                } else {
                    //lines just wants left and right edges
                    Vector2d right = project(landingSpot + panelEdges[0], pitch); //gives width by right x minus left x
                    Vector2d left = project(landingSpot + panelEdges[1], pitch); //gives x of rect
                    
                    lines[i].x1 = left.getX();
                    lines[i].x2 = right.getX();
                    lines[i].y1 = left.getY();
                    lines[i].y2 = right.getY(); //should be the same as left.getY()
                }
            }

        //CURVES is not implemented yet

        //maybe reticle mode changed
        updateHidden();
    }

private:
    enum class ReticleDrawMode : uint8_t
    {
        RECTANGLES = 0, // multiple rectangles of different colors, tops and bottoms of them are tops and bottoms of panels, midpoints of sides are edges of standard size panels
        LINES = 1,      // multiple of horizontal lines, widths represent panel widths, heights represent where the center of a standard panel needs to be to hit it, lines go across the center of a panel, endpoints are edges of panels
    };

    enum class ReticleSolveMode : uint8_t
    {
        FOR_HEIGHT_OFF_GROUND = 0, // uses pitch and DISTANCES away to solve for height, redraws the reticle often because pitch changes often
        FOR_PITCH = 1,             // uses AVERAGE_HEIGHT_OFF_GROUND and DISTANCES away to solve for pitch, reticle doesn't ever need redrawn because constants are constant
        FOR_DISTANCE_AWAY = 2      // uses pitch and AVERAGE_HEIGHT_OFF_GROUND to solve for distance away, redraws the reticle often because pitch changes often
    };

    static constexpr float AVERAGE_HEIGHT_OFF_GROUND = 0.07; //meters, center of panel to ground, for calculating where on screen reticle things should be

    static constexpr float PANEL_WIDTH = 0.135; //meters
    static constexpr float PANEL_HEIGHT = 0.125; //meters, height across the surface
    static constexpr float PANEL_ANGLE = 15 * PI / 180; // radians, tilt of the panel, 0 would be panel is not tilted
    Vector3d panelEdges[4] = {{ PANEL_WIDTH/2, 0, 0}, //right
                              {-PANEL_WIDTH/2, 0, 0}, //left
                              {0,  std::sin(PANEL_ANGLE) * PANEL_HEIGHT/2,  std::cos(PANEL_ANGLE) * PANEL_HEIGHT/2},  //top
                              {0, -std::sin(PANEL_ANGLE) * PANEL_HEIGHT/2, -std::cos(PANEL_ANGLE) * PANEL_HEIGHT/2}}; //bottom

    //corners aren't needed yet
    // Vector3d panelCorners[4] = {panelEdges[0] + panelEdges[2],  //right top
    //                             panelEdges[0] + panelEdges[3],  //right bottom
    //                             panelEdges[1] + panelEdges[2],  //left top
    //                             panelEdges[1] + panelEdges[3]}; //left bottom

    GraphicsContainer rectsContainer{};
    GraphicsContainer linesContainer{};
    // GraphicsContainer curvesContainer{};
    GimbalSubsystem* gimbal;

    ReticleDrawMode mode = ReticleDrawMode::LINES;

    
    //for rectangles and lines
    static constexpr int NUM_THINGS = 6;
    static constexpr float DISTANCES[NUM_THINGS] = {1, 2, 3, 4, 5, 6}; //y distances, in meters
    static constexpr UISubsystem::Color COLORS[NUM_THINGS] = {UISubsystem::Color::PURPLISH_RED, UISubsystem::Color::PINK, UISubsystem::Color::ORANGE, UISubsystem::Color::YELLOW, UISubsystem::Color::GREEN, UISubsystem::Color::CYAN};

    UnfilledRectangle rects[NUM_THINGS];
    Line lines[NUM_THINGS];

    void updateHidden() {
        //maybe make it so that objects at the same index have the same graphics name, so instead of hide one show another replace
        //would require an ExclusiveContainer or SelectionContainer, each has one graphics name and many graphics objects, and you can set which is selected
        //but for now assume reticle doesn't change mode very often (if ever)
        rectsContainer.setHidden(mode!=ReticleDrawMode::RECTANGLES); //hidden if not rects
        linesContainer.setHidden(mode!=ReticleDrawMode::LINES); //hidden if not lines
        // curvesContainer.setHidden(mode!=ReticleDrawMode::CURVES);
    }

    Vector2d project(Vector3d in, float pitch){
        Vector3d temp;
        Vector3d temp2;
        
        //given in pivot, rotate by pitch
        temp = in.rotatePitch(pitch);
        //now that we rotated around pivot, go to vtm
        temp2 = Projections::pivotSpaceToVtmSpace(temp);
        //now get to screen space
        return Projections::vtmSpaceToScreenSpace(temp2);
    }
};