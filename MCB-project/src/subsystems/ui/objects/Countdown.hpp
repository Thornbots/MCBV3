#pragma once

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/AtomicGraphicsObjects.hpp"

using namespace subsystems;

//when trying to buy projectiles as soon as the match starts, you can't see the original countdown
//this is drawn to the side so you can still know the countdown
class Countdown : public GraphicsContainer {
public:
    Countdown(src::Drivers* drivers) : drivers(drivers) {
        addGraphicsObject(&number);
        number.x = X_POSITION;
        number.y = Y_POSITION;
        number.height = LINE_HEIGHT;
    }

    void update() {
        number.integer = drivers->refWrapper.getStageTime();
        number.calculateNumbers();
        number.x = X_POSITION - number.width/2;

        if(drivers->refWrapper.inInitialization()){
            number.color = UISubsystem::Color::ORANGE;
            number.show();
        } else if(drivers->refWrapper.inCountdown()){
            number.color = UISubsystem::Color::CYAN;
            number.show();
        } else {
            number.hide();
        }
    }

private:
    src::Drivers* drivers;

    static constexpr uint16_t X_POSITION = 1680; //pixels, all numbers at the same y level on screen
    static constexpr uint16_t Y_POSITION = 610; //pixels, all numbers at the same y level on screen
    static constexpr uint16_t LINE_HEIGHT = 200; //pixels, this is a large number

    IntegerGraphic number{};
};