#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace subsystems;

class PeekingLines : public GraphicsContainer {
public:
    PeekingLines(DrivetrainSubsystem* drivetrain) : drivetrain(drivetrain) {
        addGraphicsObject(&line);
    }

    void update() {
        if(drivetrain->isPeekingLeft)
            setLineLeft();
        else if(drivetrain->isPeekingRight)
            setLineRight();
        else
            setLineNone();

        line.color = drivetrain->isAtPeekTarget ? UISubsystem::Color::CYAN : UISubsystem::Color::BLACK;
    }

private:
    DrivetrainSubsystem* drivetrain;

    // distance from center might be robot dependent
    static constexpr uint16_t DISTANCE_FROM_CENTER = 100; //x distance: 0 would make the peeking lines in the center of the screen
    static constexpr uint16_t BOTTOM_OFFSET = 480; //y distance from the end of the line to the bottom of the screen
    static constexpr uint16_t TOP_OFFSET = 320; //y distance from the top of the line to the top of the screen
    static constexpr uint16_t THICKNESS = 2; //pixels

    Line line{UISubsystem::Color::CYAN, 0, 0, 0, 0, THICKNESS};

    void setLineLeft() {
        line.x1 = UISubsystem::HALF_SCREEN_WIDTH - DISTANCE_FROM_CENTER;
        line.y1 = BOTTOM_OFFSET;
        line.x2 = line.x1;
        line.y2 = UISubsystem::SCREEN_HEIGHT - TOP_OFFSET;
    }

    void setLineRight() {
        line.x1 = UISubsystem::HALF_SCREEN_WIDTH + DISTANCE_FROM_CENTER;
        line.y1 = BOTTOM_OFFSET;
        line.x2 = line.x1;
        line.y2 = UISubsystem::SCREEN_HEIGHT - TOP_OFFSET;
    }

    void setLineNone() {
        line.x1 = 0;
        line.x2 = 0;
        line.y1 = 0;
        line.y2 = 0;
    }
};