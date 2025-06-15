#pragma once

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace subsystems;

class LinearVelocityIndicator : public GraphicsContainer {
public:
    LinearVelocityIndicator(DrivetrainSubsystem* drivetrain) : drivetrain(drivetrain) {
        addGraphicsObject(&number);
        number.x = CENTER_X;
        number.y = POSITION_Y;
        number.height = HEIGHT;

        //find avg location between 2 characters and 3, to put the decimal point where it needs to go
        number.integer = 12;
        number.calculateNumbers();
        decimalXOffset = number.width;
        
        number.integer = 123;
        number.calculateNumbers();
        decimalXOffset += number.width;
        decimalXOffset/=2;

        decimal.cy = POSITION_Y + THICKNESS/2;
        decimal.r = THICKNESS / 2;
        decimal.thickness = THICKNESS;
    }

    void update() {
        number.integer = drivetrain->linearVelocityMultiplierTimes100;
        number.calculateNumbers();
        number.x = CENTER_X+number.width/2;

        decimal.cx = number.x + number.width - decimalXOffset;

        if(drivetrain->isBeyblading){
            number.show();
            number.color = UISubsystem::Color::GREEN;
            decimal.color = UISubsystem::Color::GREEN;
        }  else {
            number.hide();
            number.color = UISubsystem::Color::ORANGE;
            decimal.color = UISubsystem::Color::ORANGE;
        }
    }

private:
    DrivetrainSubsystem* drivetrain;

    static constexpr uint16_t CENTER_X = 1393;   // pixels, specific to line up with fire rate and projectile allowance
    static constexpr uint16_t POSITION_Y = 620;  // pixels, to be above fire rate and projectile allowance
    static constexpr uint16_t HEIGHT = 20;       // pixels
    static constexpr uint16_t THICKNESS = 2;     // pixels

    uint16_t decimalXOffset;

    IntegerGraphic number;
    UnfilledCircle decimal; //decimal point to pretend to be a float
};