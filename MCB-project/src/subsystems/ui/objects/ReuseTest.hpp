#pragma once

#include "subsystems/servo/ServoSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/AtomicGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class ReuseTest : public GraphicsContainer {
public:
    ReuseTest(tap::Drivers* drivers) : drivers(drivers) {
        addGraphicsObject(&square);
        addGraphicsObject(&circle);
        addGraphicsObject(&string);
    }

    void update() {
        bool f = drivers->remote.keyPressed(Remote::Key::F);
        if(prev && !f) {
            prev=false;
        }
        if(!prev && f){
            prev=true;
            i++;
            if(i==3) i=0;
        }
        
        square.setHidden(i!=0);
        circle.setHidden(i!=1);
        string.setHidden(i!=2);
    }

private:
    tap::Drivers* drivers;
    
    bool prev = false;
    
    int i=0;

    UnfilledRectangle square{UISubsystem::Color::WHITE, 1000, 600, 20, 20, 5};
    UnfilledCircle circle{UISubsystem::Color::WHITE, 1000, 700, 20, 5};
    StringGraphic string{UISubsystem::Color::WHITE, "Str", UISubsystem::HALF_SCREEN_WIDTH, 650, 20, 3};

};