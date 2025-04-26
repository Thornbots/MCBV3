#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class TestGraphics : public GraphicsContainer {
public:
    TestGraphics() {
        bounds1Before.color = RefSerialData::Tx::GraphicColor::PINK;
        bounds2Before.color = RefSerialData::Tx::GraphicColor::PINK;
        bounds3Before.color = RefSerialData::Tx::GraphicColor::PINK;

        int1.outputRect(&bounds1After);
        // string2.outputRect(&bounds2After);
        // float3.outputRect(&bounds3After);
        
        addGraphicsObject(&arc1);

        addGraphicsObject(&line1);
        addGraphicsObject(&line2);
        addGraphicsObject(&rect1);
        addGraphicsObject(&rect3);
        addGraphicsObject(&bounds2Before);
        // addGraphicsObject(&string2);       // should appear before any of the ones before (assuming TestGraphics is first in its container)
        addGraphicsObject(&bounds2After);  // seventh object

        addGraphicsObject(&bounds1Before);
        addGraphicsObject(&int1);  // should appear with the other ones
        addGraphicsObject(&bounds1After);

        addGraphicsObject(&arc2); // this one updates

        addGraphicsObject(&bounds3Before);
        // addGraphicsObject(&float3);  // should appear with the other ones
        addGraphicsObject(&bounds3After);// seventh object

        addGraphicsObject(&circ1); 
        addGraphicsObject(&circ2); 
        addGraphicsObject(&ell1);

        
        addGraphicsObject(&rect2);
    }

    void update() {
        if (angle == 360) angle = 0;
        angle++;
        arc2.startAngle = angle;
        arc2.endAngle = angle + 30;
    }

private:
    Line line1{RefSerialData::Tx::GraphicColor::CYAN, 20, 700, 70, 750, 5};
    Line line2{RefSerialData::Tx::GraphicColor::CYAN, 80, 700, 80, 750, 5};
    UnfilledRectangle rect1{RefSerialData::Tx::GraphicColor::CYAN, 100, 700, 50, 50, 1};
    UnfilledRectangle rect2{RefSerialData::Tx::GraphicColor::CYAN, 170, 700, 50, 50, 10};
    UnfilledRectangle rect3{RefSerialData::Tx::GraphicColor::CYAN, 100, 770, 50, 50, 10};

    UnfilledRectangle bounds1Before{RefSerialData::Tx::GraphicColor::ORANGE, 100, 550, 100, 100, 1};
    IntegerGraphic int1{4259, &bounds1Before};
    UnfilledRectangle bounds1After{RefSerialData::Tx::GraphicColor::GREEN, 100, 550, 100, 100, 1};

    UnfilledRectangle bounds2Before{RefSerialData::Tx::GraphicColor::ORANGE, 500, 550, 100, 100, 1};
    // StringGraphic string2{"Test", &bounds2Before};
    UnfilledRectangle bounds2After{RefSerialData::Tx::GraphicColor::PURPLISH_RED, 500, 550, 100, 100, 1};

    UnfilledRectangle bounds3Before{RefSerialData::Tx::GraphicColor::ORANGE, 1100, 550, 100, 100, 1};
    // FloatGraphic float3{8.12345f, &bounds3Before};
    UnfilledRectangle bounds3After{RefSerialData::Tx::GraphicColor::PURPLISH_RED, 1100, 550, 100, 100, 1};

    UnfilledCircle circ1{RefSerialData::Tx::GraphicColor::CYAN, 1800, 300, 25, 1};
    UnfilledCircle circ2{RefSerialData::Tx::GraphicColor::CYAN, 1800, 600, 25, 5};
    Arc arc1{RefSerialData::Tx::GraphicColor::CYAN, 100, 700, 50, 50, 0, 30, 1};
    UnfilledEllipse ell1{RefSerialData::Tx::GraphicColor::CYAN, 1600, 500, 100, 50, 1};

    uint16_t angle = 0;
    Arc arc2{RefSerialData::Tx::GraphicColor::YELLOW, 170, 770, 50, 50, 0, 30, 1};
};