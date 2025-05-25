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

private:
    enum class ReticleMode : uint8_t
    {
        // LINES = 0,      // multiple of horizontal lines, widths represent panel widths, heights represent where the center of a standard panel needs to be to hit it, lines go across the center of a panel, endpoints are edges of panels
        // CURVES = 1,     // two curves that act like the endpoints of a lot of LINES (see above)
        RECTANGLES = 2, // multiple rectangles of different colors, tops and bottoms of them are tops and bottoms of panels, midpoints of sides are edges of panels
    };

    GraphicsContainer lines{};
    GraphicsContainer curves{};
    GraphicsContainer rects{};

    static constexpr float DISTANCES[6] = {0.5, 1, 2, 5, 9, 12};

    //for rectangles
    static constexpr UISubsystem::Color COLORS[6] = {UISubsystem::Color::PURPLISH_RED, UISubsystem::Color::PINK, UISubsystem::Color::ORANGE, UISubsystem::Color::YELLOW, UISubsystem::Color::GREEN, UISubsystem::Color::CYAN};

};