#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class Reticle : public GraphicsContainer {
public:
    Reticle() {
        // addGraphicsObject(&left);
        // addGraphicsObject(&right);
    }

    void update() final {
        
    }

private:
    

    // Line left{RefSerialData::Tx::GraphicColor::CYAN, static_cast<uint16_t>(HALF_WIDTH - BOTTOM_OFFSET), 0, static_cast<uint16_t>(HALF_WIDTH - TOP_OFFSET), HEIGHT, THICKNESS};
    // Line right{RefSerialData::Tx::GraphicColor::CYAN, static_cast<uint16_t>(HALF_WIDTH + BOTTOM_OFFSET), 0, static_cast<uint16_t>(HALF_WIDTH + TOP_OFFSET), HEIGHT, THICKNESS};
};