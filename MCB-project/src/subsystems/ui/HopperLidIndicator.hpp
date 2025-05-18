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
            lid.y1 = 810;
            lid.y2 = 810;
        } else {
            lid.x1 = 650;
            lid.y1 = 790;
            lid.y2 = 790;
        }
        lid.color = servo->isAtTarget() ? UISubsystem::Color::GREEN : UISubsystem::Color::ORANGE;
    }

private:
    ServoSubsystem* servo;
    
    Line lid{UISubsystem::Color::CYAN, 690, 810, 670, 810, 20};
    UnfilledRectangle frame{UISubsystem::Color::WHITE, 670, 810, 20, 20, 2};
};