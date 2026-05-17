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
        addGraphicsObject(&o1);
        addGraphicsObject(&o2);
        addGraphicsObject(&o3);
        addGraphicsObject(&o4);
        addGraphicsObject(&o5);
        setHidden(i);
    }

    void update() {
        bool f = drivers->remote.keyPressed(Remote::Key::F);
        if(prev && !f) {
            prev=false;
        }
        if(!prev && f){
            prev=true;
            i++;
            if(i==5) i=0;
            setHidden(i);
        }
    }

private:
    tap::Drivers* drivers;
    
    void setHidden(int i){
        o1.setHidden(i!=0);
        o2.setHidden(i!=1);
        o3.setHidden(i!=2);
        o4.setHidden(i!=3);
        o5.setHidden(i!=4);
    }
    
    bool prev = false;
    
    int i=0;

    IntegerGraphic o1{UISubsystem::Color::WHITE, 1, 10, 600, 20, 5};
    // UnfilledCircle o2{UISubsystem::Color::WHITE, 1000, 700, 20, 5};
    // StringGraphic o3{UISubsystem::Color::WHITE, "Str", UISubsystem::HALF_SCREEN_WIDTH, 650, 20, 3};
    
    // IntegerGraphic o3{UISubsystem::Color::WHITE, 123, UISubsystem::HALF_SCREEN_WIDTH, 650, 20, 3};
    
    IntegerGraphic o2{UISubsystem::Color::WHITE, 2, 10, 600, 20, 5};
    IntegerGraphic o3{UISubsystem::Color::WHITE, 3, 10, 600, 20, 5};
    IntegerGraphic o4{UISubsystem::Color::WHITE, 4, 10, 600, 20, 5};
    IntegerGraphic o5{UISubsystem::Color::WHITE, 5, 10, 600, 20, 5};
    
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