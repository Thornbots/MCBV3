#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class Reticle : public GraphicsContainer {
public:
    Reticle() {
        addGraphicsObject(&rects);
        addGraphicsObject(&lines);
        addGraphicsObject(&curves);

        updateHidden();
    }

private:
    enum class ReticleMode : uint8_t
    {
        RECTANGLES = 0, // multiple rectangles of different colors, tops and bottoms of them are tops and bottoms of panels, midpoints of sides are edges of standard size panels
        // LINES = 1,      // multiple of horizontal lines, widths represent panel widths, heights represent where the center of a standard panel needs to be to hit it, lines go across the center of a panel, endpoints are edges of panels
        // CURVES = 2,     // two curves that act like the endpoints of a lot of LINES (see above)
    };

    GraphicsContainer rects{};
    GraphicsContainer lines{};
    GraphicsContainer curves{};

    ReticleMode mode = ReticleMode::RECTANGLES;

    static constexpr float DISTANCES[6] = {0.5, 1, 2, 5, 9, 12};

    //for rectangles
    static constexpr UISubsystem::Color COLORS[6] = {UISubsystem::Color::PURPLISH_RED, UISubsystem::Color::PINK, UISubsystem::Color::ORANGE, UISubsystem::Color::YELLOW, UISubsystem::Color::GREEN, UISubsystem::Color::CYAN};


    void updateHidden() {

    }
};