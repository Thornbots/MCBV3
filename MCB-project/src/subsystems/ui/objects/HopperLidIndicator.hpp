#pragma once

#include "subsystems/servo/ServoSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/AtomicGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class HopperLidIndicator : public GraphicsContainer {
public:
    HopperLidIndicator(ServoSubsystem* servo) : servo(servo) {
        addGraphicsObject(&lid);
        addGraphicsObject(&frame);
    }

    void update() {
        if (servo->getTargetIsClosed()) {
            lid.x2 = CENTER_X + SIZE;
            lid.y1 = CENTER_Y + SIZE / 2;
        } else {
            lid.x2 = CENTER_X - SIZE;
            lid.y1 = CENTER_Y - SIZE / 2;
        }
        lid.y2 = lid.y1;
    }

private:
    ServoSubsystem* servo;

    Line lid{UISubsystem::Color::GREEN, CENTER_X, 0, 0, 0, SIZE};
    UnfilledRectangle frame{UISubsystem::Color::WHITE, CENTER_X - FRAME_THICKNESS / 2, CENTER_Y - FRAME_THICKNESS / 2, SIZE + FRAME_THICKNESS, SIZE + FRAME_THICKNESS, FRAME_THICKNESS};

    static constexpr uint16_t CENTER_X = 1250;  // puts it on top of )
    static constexpr uint16_t CENTER_Y = 810;   // puts it on top of )
    
    static constexpr uint16_t SIZE = 20;
    static constexpr uint16_t FRAME_THICKNESS = 2;
};