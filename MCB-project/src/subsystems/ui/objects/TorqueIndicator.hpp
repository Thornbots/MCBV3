#pragma once

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/AtomicGraphicsObjects.hpp"

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"
#include "tap/control/subsystem.hpp"


using namespace subsystems;

class TorqueIndicator : public GraphicsContainer {
public:
    TorqueIndicator(src::Drivers* drivers, tap::motor::DjiMotor* motor, uint8_t index) : 
    drivers(drivers),
    motor(motor),
    index(index) {
        addGraphicsObject(&number);
        if(index == 1){
            number.x = 1070;
            number.y = 360;
        } else if(index == 2){
            number.x = 660;
            number.y = 360;
        } else if(index == 3){
            number.x = 1070;
            number.y = 670;
        } else if(index == 4){
            number.x = 660;
            number.y = 670;
        }
    }

    void update() {
        int16_t curvalue = motor->getTorque();
        
        if(drivers->remote.keyPressed(Remote::Key::R)) {
            maxvalue = curvalue;
        }
        
        int16_t absmax = maxvalue<0 ? -maxvalue : maxvalue;
        int16_t abscur = curvalue<0 ? -curvalue : curvalue;
        
        if(abscur>absmax) maxvalue = curvalue;
        
        number._float = maxvalue;
        
        number.color = motor->isMotorOnline() ? UISubsystem::Color::GREEN : UISubsystem::Color::PINK;
    }

private:
    src::Drivers* drivers;
    tap::motor::DjiMotor* motor;
    uint8_t index;
    int16_t maxvalue = 0;

    static constexpr uint16_t X_POSITION = 1680; //pixels, all numbers at the same y level on screen
    static constexpr uint16_t Y_POSITION = 610; //pixels, all numbers at the same y level on screen

    FloatGraphic number{UISubsystem::Color::GREEN, 0, 0, 0, 60, 3};
};