#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/AtomicGraphicsObjects.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"

using namespace subsystems;

class SupercapChargeIndicator : public GraphicsContainer {
public:
    SupercapChargeIndicator(DrivetrainSubsystem* drivetrain) : drivetrain(drivetrain) {
        addGraphicsObject(&arc);
        update();
    }

    void update() {
        arc.setHigher((getCurrentCharge() - MIN_CHARGE) / (MAX_CHARGE - MIN_CHARGE));
        arc.color = getCurrentColor();
    }

private:
    DrivetrainSubsystem* drivetrain;

    static constexpr float MIN_CHARGE = 20;    // joules
    static constexpr float MAX_CHARGE = 1600;  // joules
    float getCurrentCharge() {
        // needs to get from actual supercaps at some point
        return MAX_CHARGE;
    }

    UISubsystem::Color getCurrentColor() {
        //cyan (energy-colored) if in keyboard mode, pink if in controller mode, black if neither (controller is probably off)
        return drivetrain->isInKeyboardMode ? UISubsystem::Color::CYAN : drivetrain->isInControllerMode ? UISubsystem::Color::PINK : UISubsystem::Color::BLACK;
    }

    LargeCenteredArc arc{true, 0};  // arc on the left size in first lane
};