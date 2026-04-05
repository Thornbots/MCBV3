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
        setHidden(0);
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
            setHidden(i);
        }
    }

private:
    tap::Drivers* drivers;
    
    void setHidden(int i){
        square.setHidden(i!=0);
        circle.setHidden(i!=1);
        string.setHidden(i!=2);
    }
    
    bool prev = false;
    
    int i=0;

    UnfilledRectangle square{UISubsystem::Color::WHITE, 1000, 600, 20, 20, 5};
    UnfilledCircle circle{UISubsystem::Color::WHITE, 1000, 700, 20, 5};
    StringGraphic string{UISubsystem::Color::WHITE, "Str", UISubsystem::HALF_SCREEN_WIDTH, 650, 20, 3};

};



class ReuseTest2 : public GraphicsContainer {
public:
    ReuseTest2(tap::Drivers* drivers) : drivers(drivers) {
        for(int i=0; i<7; i++){
            squareContainer.addGraphicsObject(squares+i);
            circleContainer.addGraphicsObject(circles+i);
            squares[i].x = 700+20*i;
            squares[i].y = 700;
            squares[i].width=(squares[i].height=10);
            squares[i].thickness=(circles[i].r=(circles[i].thickness=5));
            circles[i].cx=705+20*i;
            circles[i].cy=725;
        }
        addGraphicsObject(&squareContainer);
        addGraphicsObject(&circleContainer);
        setHidden(0);
    }

    void update() {
        bool f = drivers->remote.keyPressed(Remote::Key::F);
        if(prev && !f) {
            prev=false;
        }
        if(!prev && f){
            prev=true;
            i++;
            if(i==2) i=0;
            setHidden(i);
        }
    }

private:
    tap::Drivers* drivers;
    
    void setHidden(int i){
        squareContainer.setHidden(i!=0);
        circleContainer.setHidden(i!=1);
    }
    
    bool prev = false;
    
    int i=0;

    UnfilledRectangle squares[7];
    UnfilledCircle circles[7];
    GraphicsContainer squareContainer;
    GraphicsContainer circleContainer;
};