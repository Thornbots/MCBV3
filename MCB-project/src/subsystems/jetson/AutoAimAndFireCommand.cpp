#include "AutoAimAndFireCommand.hpp"

namespace commands {
using namespace tap::communication::serial;

void AutoAimAndFireCommand::initialize() {
    shoot = -1;
    isScheduled = true;
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

        //don't spin before match
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

        pitch = currentPitch + (pitch - currentPitch) / 5.0f; //here tune pitch
        if (abs(dyaw) > .05) {
            dyaw /= 4.0f;}
        else dyaw /= 1.75;
        //here tune yaw
        if (allowGimbal) gimbal->updateMotorsAndVelocityWithLatencyCompensation(dyaw/20.0f, pitch, yawvel, pitchvel);  // division is to prevent overshoot from latency
        if (shoot == 1) isShooting = true;
    } else if (tap::arch::clock::getTimeMilliseconds() - lastSeenTime < PERSISTANCE) {
        //Haven't found a target right now but I have recently, keep shooting if I was shooting

        if(allowGimbal) gimbal->updateMotors(0, pitch);
    } else {
        //Haven't found a target, patrol

        isShooting = false;
        // pitch = 0.05;  // pitch down to avoid looking into the sky
        numCyclesForBurst++;

        if(allowGimbal) {
            if (numCyclesForBurst == CYCLES_UNTIL_BURST) {
                gimbal->updateMotors(BURST_AMOUNT, 0);
                numCyclesForBurst = 0;
            } else {
                gimbal->updateMotors(PATROL_SPEED, 0);
            }
        }
    }

    if(allowShooting){
        if (isShooting) {
            // if we see a panel or recently have seen a panel
            indexer->indexAtRate(5);//20 change to not make a mess
        } else {
            // if we haven't seen a panel for a bit
             indexer->stopIndex();
            // indexer->unjam();
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
    isScheduled = false;
}

bool AutoAimAndFireCommand::getIsScheduled() { return isScheduled; }


bool AutoAimAndFireCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands