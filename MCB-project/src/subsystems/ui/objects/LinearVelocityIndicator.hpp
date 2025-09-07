#pragma once

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/AtomicGraphicsObjects.hpp"

using namespace subsystems;

class LinearVelocityIndicator : public GraphicsContainer {
public:
    LinearVelocityIndicator(DrivetrainSubsystem* drivetrain) : drivetrain(drivetrain) {
        addGraphicsObject(&number);
        
        number.x = CENTER_X;
        number.y = POSITION_Y;
        number.height = HEIGHT;
    }

    void update() {
        number.integer = drivetrain->linearVelocityMultiplierTimes100;
        number.calculateNumbers();
        number.x = CENTER_X - number.width/2;

        if(drivetrain->isBeyblading){
            number.show();
            number.color = UISubsystem::Color::GREEN;
        }  else {
            number.hide();
            number.color = UISubsystem::Color::ORANGE;
        }
    }

private:
    DrivetrainSubsystem* drivetrain;

    static constexpr uint16_t CENTER_X = 1393;   // pixels, specific to line up with fire rate and projectile allowance
    static constexpr uint16_t POSITION_Y = 620;  // pixels, to be above fire rate and projectile allowance
    static constexpr uint16_t HEIGHT = 20;       // pixels
    static constexpr uint16_t THICKNESS = 2;     // pixels

    IntegerGraphic number;
};