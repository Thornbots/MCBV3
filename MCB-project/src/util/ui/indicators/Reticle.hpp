#pragma once

#include "util/ui/UIScheduler.hpp"

using namespace tap::communication::serial;
using namespace ui;

class Reticle : public GraphicsContainer {
public:
    Reticle() {
        // addGraphicsObject(&left);
        // addGraphicsObject(&right);
    }

private:
    

    // Line left{RefSerialData::Tx::GraphicColor::CYAN, static_cast<uint16_t>(HALF_WIDTH - BOTTOM_OFFSET), 0, static_cast<uint16_t>(HALF_WIDTH - TOP_OFFSET), HEIGHT, THICKNESS};
    // Line right{RefSerialData::Tx::GraphicColor::CYAN, static_cast<uint16_t>(HALF_WIDTH + BOTTOM_OFFSET), 0, static_cast<uint16_t>(HALF_WIDTH + TOP_OFFSET), HEIGHT, THICKNESS};
};