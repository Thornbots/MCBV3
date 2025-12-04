#include "AutoAimAndFireCommand.hpp"
#include <cmath>

namespace commands {
using namespace tap::communication::serial;

void AutoAimAndFireCommand::initialize() {
    shoot = -1;

    drivers->leds.set(tap::gpio::Leds::Green, true);
}
void AutoAimAndFireCommand::execute() {
    bool allowShooting = true;
    bool allowGimbal = true;

    if (drivers->refSerial.getRefSerialReceivingData() && 
       (drivers->refSerial.getGameData().gameType == RefSerialData::Rx::GameType::ROBOMASTER_RMUL_3V3)) {

        allowShooting = false;
        allowGimbal = false;

        if (drivers->refSerial.getGameData().gameStage == RefSerialData::Rx::GameStage::IN_GAME) {
            // allow both
            allowShooting = true;
            allowGimbal = true;
        }

        if (drivers->refSerial.getGameData().gameStage == RefSerialData::Rx::GameStage::COUNTDOWN) {
            // countdown, only allow gimbal
            allowGimbal = true;
        }

    }


    float dyaw = 0;
    float currentYaw = gimbal->getYawAngleRelativeWorld();
    float currentPitch = gimbal->getPitchEncoderValue();
    cv->update(currentYaw, currentPitch, yawvel, pitchvel, &dyaw, &pitch, &yawvel, &pitchvel, &shoot);

    if (shoot != -1) {
        //if(tap::arch::clock::getTimeMilliseconds() - lastSeenTime >  PERSISTANCE) flip = flip * -1;
        //Found a target, moving to it and maybe shooting at it

        dyaw = fmod(dyaw, 2 * PI);
        // clamp between -Pi and PI to allow for dividing
        dyaw = dyaw > PI ? dyaw - 2 * PI : dyaw < -PI ? dyaw + 2 * PI : dyaw;
        lastSeenTime = tap::arch::clock::getTimeMilliseconds();

        float newPitch = currentPitch + (pitch - currentPitch) / 4.0f;
        if (abs(dyaw) > .05) {
            dyaw /= 4.0f;}
        else dyaw /= 1.75;
        if (allowGimbal) gimbal->updateMotorsAndVelocityWithLatencyCompensation(dyaw/2.5f, pitch, yawvel, pitchvel);  // division is to prevent overshoot from latency
        if (shoot == 1) isShooting = true;
    } else if (tap::arch::clock::getTimeMilliseconds() - lastSeenTime < PERSISTANCE) {
        //Haven't found a target right now but I have recently, keep shooting if I was shooting

        if(allowGimbal) gimbal->updateMotors(0, pitch);
    } else {
        //Haven't found a target, patrol

        cycle++;
        isShooting = false;
        pitch = 0.2 + 0.15 * std::sin(cycle * 2 * PI / PITCH_CYCLES);  // pitch down to avoid looking into the sky
        numCyclesForBurst++;

        if(allowGimbal) {
            if (numCyclesForBurst == CYCLES_UNTIL_BURST) {
                gimbal->updateMotors(BURST_AMOUNT, pitch);
                numCyclesForBurst = 0;
            } else {
                gimbal->updateMotors(PATROL_SPEED, pitch);
            }
        }
    }

    if(allowShooting){
        if (isShooting) {
            // if we see a panel or recently have seen a panel
            indexer->indexAtRate(20);
        } else {
            // if we haven't seen a panel for a bit
            //  indexer->stopIndex();
            indexer->unjam();
        }
    } else {
        indexer->stopIndex();
        
    }

    if(allowGimbal) {
        flywheel->setTargetVelocity(FLYWHEEL_MOTOR_MAX_RPM);
    } else {
        gimbal->stopMotors();
        if(adc->getIsScheduled()) flywheel->setTargetVelocity(FLYWHEEL_MOTOR_MAX_RPM/4);
    }
}

void AutoAimAndFireCommand::end(bool) {
    pitch = 0;
    drivers->leds.set(tap::gpio::Leds::Green, false);
}

bool AutoAimAndFireCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands