#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

#include "subsystems/servo/ServoSubsystem.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class HopperLidIndicator : public GraphicsContainer {
public:
    HopperLidIndicator(ServoSubsystem* servo) : servo(servo) {
        addGraphicsObject(&lid);
        addGraphicsObject(&frame);
    }

    void update() {
        if(servo->getTargetIsClosed()){
            lid.x1 = 690;
            lid.y1 = 820;
            lid.y2 = 820;
        } else {
            lid.x1 = 650;
            lid.y1 = 800;
            lid.y2 = 800;
        }
    }

private:
    ServoSubsystem* servo;
    
    Line lid{UISubsystem::Color::GREEN, 690, 820, 670, 820, 20};
    UnfilledRectangle frame{UISubsystem::Color::WHITE, 670, 810, 20, 20, 2};
};