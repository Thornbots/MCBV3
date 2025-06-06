#pragma once

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace subsystems;

// looks like
//    __
//   /
//
//
// if someone hit you in that direction
class HitRing : public GraphicsContainer {
public:
    HitRing(GimbalSubsystem* gimbal) : gimbal(gimbal) {
        for(int i = 0; i<NUM_HISTORY; i++){
            rings[i].color = UISubsystem::Color::PURPLISH_RED;
            rings[i].width = STARTING_SIZE + SIZE_INCREMENT*i;
            rings[i].height = rings[i].width;
            rings[i].cx = UISubsystem::HALF_SCREEN_WIDTH;
            rings[i].cy = UISubsystem::HALF_SCREEN_HEIGHT;
        }
    }

    void update() {
        uint16_t heading = static_cast<uint16_t>(gimbal->getYawEncoderValue() * YAW_MULT + YAW_OFFSET);
        // if the gimbal compared to the drivetrain (from the encoder) is facing forward, heading would be 0, if facing right, heading would be 90

    }

    void fixAngle(uint16_t* a) {
        *a %= 360;  // set a to the remainder after dividing by 360, so if it was 361 it would now be 1
    }

private:
    GimbalSubsystem* gimbal;

    static constexpr uint16_t THICKNESS = 2;        // pixels
    static constexpr uint16_t ARC_LEN = 5;         // degrees
    static constexpr uint16_t STARTING_SIZE = 190;  // pixels
    static constexpr uint16_t SIZE_INCREMENT = 5;  // pixels. If 0, the history of lines will all be overlapping, 

    static constexpr float YAW_MULT = 180 / PI;  // turns radians from gimbal's getYawEncoderValue into degrees, might need to be negative
    static constexpr float YAW_OFFSET = 360;     // degrees, 0 from the yaw might not be top on the screen, also needs to make sure it is positive because we are using uints

    static constexpr int NUM_HISTORY = 5;  // how many shots to keep track of
    Arc rings[NUM_HISTORY];

    static constexpr uint32_t EXPIRATION_TIME = 2000;         // once hit, it shows for 2 seconds
    tap::arch::MilliTimeout expirationTimeouts[NUM_HISTORY];  // for knowing when an old hit expires
};