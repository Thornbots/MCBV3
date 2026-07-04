#include "AutoAimAndFireCommand.hpp"
#include "JetsonSubsystemConstants.hpp"


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
    float currentpitchvel = gimbal->getPitchVel(); //did add this for the actual CV stuff can make this 0 if we want
    cv->update(currentYaw, currentPitch, yawvel, currentpitchvel, &dyaw, &pitch, &yawvel, &pitchvel, &shoot);

    tap::communication::serial::RefSerial::Rx::RobotData robotData = drivers->refSerial.getRobotData();
    bool inRfid = robotData.rfidStatus.all(tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::RESUPPLY_ZONE_OUTSIDE_EXCHANGE);
    if(inRfid && allowGimbal){ //am i over the rfid
        gimbal->setAngles(0, 0);
        newPitch = 0;
    } else if (shoot != -1) {
        //if(tap::arch::clock::getTimeMilliseconds() - lastSeenTime >  PERSISTANCE) flip = flip * -1;
        //Found a target, moving to it and maybe shooting at it

        dyaw = fmod(dyaw, 2 * PI);
        // clamp between -Pi and PI to allow for dividing
        dyaw = dyaw > PI ? dyaw - 2 * PI : dyaw < -PI ? dyaw + 2 * PI : dyaw;
        lastSeenTime = tap::arch::clock::getTimeMilliseconds();

        // pitch damper (analogous to the yaw divisor below): only step a fraction of the way
        // from the current pitch toward the CV target each cycle, so the setpoint eases in
        // instead of jumping with every (latent, ~30Hz) CV frame. Larger divisor = more
        // damping / smoother but more lag; smaller = snappier but can ring.
        float dpitch = pitch - currentPitch;

        newPitch = 0.10; //currentPitch + dpitch * 0.2;//std::clamp(std::abs(dpitch)*PITCH_MULTIPLY_SCALE, PITCH_MULTIPLY_MIN, PITCH_MULTIPLY_MAX);

        dyaw *= std::clamp(std::abs(dyaw)*YAW_MULTIPLY_SCALE, YAW_MULTIPLY_MIN, YAW_MULTIPLY_MAX);
        //WithLatencyCompensation
        if (allowGimbal) gimbal->updateMotorsAndVelocity(dyaw, newPitch, yawvel, pitchvel);  // division is to prevent overshoot from latency
        if (shoot == 1) isShooting = true;
    } else if (tap::arch::clock::getTimeMilliseconds() - lastSeenTime < PERSISTANCE) {
        //Haven't found a target right now but I have recently, keep shooting if I was shooting

        if(allowGimbal) gimbal->updateMotors(0, newPitch);
    } else {
        //Haven't found a target, patrol

        isShooting = false;
        newPitch = 0.05;  // pitch down to avoid looking into the sky
        numCyclesForBurst++;

        if(allowGimbal) {
            // getAngleToTurnForSentry() returns HitRing::PLACEHOLDER_ANGLE except on the single
            // cycle right after a hit is registered, when it returns (headYaw - hitDirection) in
            // world radians. Latch that one-shot value into an absolute world-yaw target and hold
            // it, otherwise it is lost the instant patrol resumes and the turret never turns.
            float angleToTurnForSentry = cv->getAngleToTurnForSentry();
            if (angleToTurnForSentry != HitRing::PLACEHOLDER_ANGLE) {
                // Face the hit: target heading = current heading minus the returned offset.
                // (If the turret turns AWAY from the hit on hardware, flip this sign to a +.)
                hitTargetYaw = currentYaw - angleToTurnForSentry;
                turningToHit = true;
                hitTurnStartTime = tap::arch::clock::getTimeMilliseconds();
            }

            if (turningToHit && tap::arch::clock::getTimeMilliseconds() - hitTurnStartTime < HIT_TURN_DURATION) {
                // Hold the heading toward the hit. CV still runs at the top of execute(), so if the
                // attacker comes into view the shoot branch takes over and engages it.
                gimbal->setAngles(hitTargetYaw, newPitch);
            } else {
                turningToHit = false;
                if (numCyclesForBurst == CYCLES_UNTIL_BURST) {
                    gimbal->updateMotors(BURST_AMOUNT, newPitch);
                    numCyclesForBurst = 0;
                } else {
                    gimbal->updateMotors(PATROL_SPEED, newPitch);
                }
            }
        }
    }

    if(allowShooting){
        if (isShooting) {
            // if we see a panel or recently have seen a panel
            indexer->indexAtRate(10);//20 change to not make a mess
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
