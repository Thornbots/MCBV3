#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace subsystems;

class Reticle : public GraphicsContainer {
public:
    Reticle() {
        // addGraphicsObject(&left);
        // addGraphicsObject(&right);
    }

private:
    

    // Line left{UISubsystem::Color::CYAN, static_cast<uint16_t>(HALF_WIDTH - BOTTOM_OFFSET), 0, static_cast<uint16_t>(HALF_WIDTH - TOP_OFFSET), HEIGHT, THICKNESS};
    // Line right{UISubsystem::Color::CYAN, static_cast<uint16_t>(HALF_WIDTH + BOTTOM_OFFSET), 0, static_cast<uint16_t>(HALF_WIDTH + TOP_OFFSET), HEIGHT, THICKNESS};
};