#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace subsystems;


class LinearVelocityIndicator : public GraphicsContainer {
public:
    LinearVelocityIndicator(DrivetrainSubsystem* drivetrain) : drivetrain(drivetrain) {
        // addGraphicsObject(&inside);
    }

    void update() {
        
    }

private:
    DrivetrainSubsystem* drivetrain;

    static constexpr uint16_t THICKNESS = 2;           // pixels


    
    // IntegerGraphic number{UISubsystem::Color::CYAN, UISubsystem::HALF_SCREEN_WIDTH - INSIDE_WIDTH / 2, HEIGHT_OFF_BOTTOM + PADDING, INSIDE_WIDTH, INSIDE_HEIGHT, THICKNESS};
};