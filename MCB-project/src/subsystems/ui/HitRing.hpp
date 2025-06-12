#pragma once

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/ui/ChassisOrientationIndicator.hpp"
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
    HitRing(tap::Drivers* drivers, GimbalSubsystem* gimbal) : drivers(drivers), gimbal(gimbal) {
        // initialize rings
        for (int i = 0; i < NUM_HISTORY; i++) {
            rings[i].width = STARTING_SIZE + SIZE_INCREMENT * i;
            rings[i].height = rings[i].width;
            rings[i].cx = UISubsystem::HALF_SCREEN_WIDTH;
            rings[i].cy = UISubsystem::HALF_SCREEN_HEIGHT;
            expirationTimeouts[i].stop();        // timers might be initialized started, we need them to be stopped until we get hit
            this->addGraphicsObject(rings + i);  // pointer math
            rings[i].hide();
        }
    }

    void update() {
        uint16_t heading = static_cast<uint16_t>(gimbal->getYawEncoderValue() * ChassisOrientationIndicator::YAW_MULT + ChassisOrientationIndicator::YAW_OFFSET + 360);
        uint16_t heading2 = static_cast<uint16_t>(gimbal->getYawAngleRelativeWorld() * ChassisOrientationIndicator::YAW_MULT + ChassisOrientationIndicator::YAW_OFFSET + 360);
        // if the gimbal compared to the drivetrain (from the encoder) is facing forward, heading would be 360, if facing right, heading would be 90

        // update/clear old hits
        bool allSlotsUnused = true;
        for (int i = 0; i < NUM_HISTORY; i++) {
            if (expirationTimeouts[i].isStopped()) continue;  // if timer not running, ignore this ring and go to the next

            // downgrade pink to purple
            if (expirationTimeouts[i].timeRemaining() > RECENT_TIME) {
                rings[i].color = UISubsystem::Color::PURPLISH_RED;
            }

            // expire and reclaim slot
            if (expirationTimeouts[i].isExpired()) {
                rings[i].hide();
                expirationTimeouts[i].stop();  // need to stop so we know to not check this one this next time
            } else {
                // don't get to reclaim slot, update rotation
                allSlotsUnused = false;

                updateRing(i, heading);
            }
        }

        //if no rings on screen, make it so the next time we get hit it starts in the center
        if (allSlotsUnused) nextIndex = 0;

        // check for a new hit
        if (drivers->refSerial.getRefSerialReceivingData()) {
            RefSerialData::Rx::RobotData robotData = drivers->refSerial.getRobotData();
            if (previousHp > robotData.currentHp && (robotData.damageType == RefSerialData::Rx::DamageType::ARMOR_DAMAGE || robotData.damageType == RefSerialData::Rx::DamageType::COLLISION)) {
                // took some sort of damage and we think we took panel damage

                // damagedArmorId==0 is forward, subtract 0*90 degrees keeps heading unchanged, so if was facing forward (heading was 360) then draw at angle 360: top
                // 1 is left, subtract 1*90 degrees to make 360 into 270: left
                // 2 is back, subtract 2*90 degrees to make 360 into 180: bottom
                // 3 is right, subtract 3*90 degrees to make 360 into 90: bottom
                // 4 is top, we don't have panels on top (yet?)
                hitOrientations[nextIndex] = heading2 - 90 * ((uint16_t) robotData.damagedArmorId);
                rings[nextIndex].show();
                expirationTimeouts[nextIndex].restart(RECENT_TIME + EXPIRATION_TIME);
                rings[nextIndex].color = UISubsystem::Color::PINK;
                updateRing(nextIndex, heading);

                //get the next index
                nextIndex++;
                if (nextIndex == NUM_HISTORY) nextIndex = 0;  // cycle back around and overwrite if we get hit really often
            }

            previousHp = robotData.currentHp;
        }
    }

private:
    tap::Drivers* drivers;
    GimbalSubsystem* gimbal;

    uint16_t previousHp;

    static constexpr uint16_t THICKNESS = 2;        // pixels
    static constexpr uint16_t ARC_LEN = 6;          // degrees
    static constexpr uint16_t STARTING_SIZE = 190;  // pixels
    static constexpr uint16_t SIZE_INCREMENT = 5;   // pixels. If 0, the history of lines will all be overlapping,

    static constexpr int NUM_HISTORY = 5;              // how many shots to keep track of
    static constexpr uint32_t RECENT_TIME = 200;       // once hit, it shows for 0.2 seconds pink
    static constexpr uint32_t EXPIRATION_TIME = 5000;  // then next it shows for 5 seconds purple

    Arc rings[NUM_HISTORY];
    int nextIndex = 0;
    tap::arch::MilliTimeout expirationTimeouts[NUM_HISTORY];  // for knowing how old a hit is, stopped if not hit recently

    // need to know which orientation (compared to gimbal) we got hit. 
    // This is a combination of which panel got hit and what angle the drivetrain is at (compared to gimbal)
    float hitOrientations[NUM_HISTORY];

    void updateRing(int i, uint16_t heading) {
        rings[i].startAngle = heading - hitOrientations[i] - ARC_LEN / 2;
        ChassisOrientationIndicator::fixAngle(&rings[i].startAngle);
        rings[i].endAngle = rings[i].startAngle + ARC_LEN;
    }
};