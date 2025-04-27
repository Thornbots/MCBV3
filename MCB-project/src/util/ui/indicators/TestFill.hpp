#pragma once

#include "util/ui/UIScheduler.hpp"

using namespace tap::communication::serial;
using namespace ui;

class TestFill : public GraphicsContainer {
public:
    TestFill() {
        // lets make each circle radius 30 (size 60), so 32 by 18 circles, 576 total

        for (int i = 0; i < UIScheduler::SCREEN_WIDTH; i += 2 * R) {
            for (int j = 0; j < UIScheduler::SCREEN_HEIGHT; j += 2 * R) {
                // using new is bad, this is just for testing
                addGraphicsObject(new UnfilledCircle(RefSerialData::Tx::GraphicColor::ORANGE, i + R, j + R, R, 5));
            }
        }
    }


private:
    static constexpr int R = 60;
};