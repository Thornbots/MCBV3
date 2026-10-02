#include "AutoAimAndFireCommand.hpp"
#include "JetsonSubsystemConstants.hpp"


namespace commands {
using namespace tap::communication::serial;

void AutoAimAndFireCommand::initialize() {
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
    
    tap::communication::serial::RefSerial::Rx::RobotData robotData = drivers->refSerial.getRobotData();
    bool inRfid = robotData.rfidStatus.all(tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::RESTORATION_ZONE) || robotData.rfidStatus.all(tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::EXCHANGE_ZONE);
    if(jetson->getCVTarget(&cvTarget)) {
        cvTargetValidTimeout.restart(TARGET_VALID_TIME);
        startShotTimeout.restart(cvTarget.delay_ms-FIRING_LATENCY_TIME);
    }
    if (allowGimbal&&!cvTargetValidTimeout.isExpired()) { //do position-based aiming
        // the tap::algorithms::ballistics::findTargetProjectileIntersection function
        // is useful for knowing how to hit a moving target, but the jetson already
        // did that work. So we just need to do simple projectile motion to aim
        // at a position that isn't moving.
        // The cvTarget xyz is in the coord frame from odometry, so if odo thinks
        // we are at (3, 4) and cvTarget says to aim at (3, 6, 0.2), we need to aim forward.
        // We adjust our aiming to hit the cvTarget as odo moves, so if we move to (3.2, 4)
        // while we are aiming, we need to look slightly to the left.
        
        // Note that for gimbal subsystem, positive pitch is downward.
        // yaw of 0 is forward (when rfid localizaition was used, 0,0 meant look forward and horizontal),
        // guessing that positive yaw is the way it should be (counterclockwise in xy plane)
        Vector2d deltaXY{cvTarget.x - odo->getX(), cvTarget.y - odo->getY()};
        targetYaw = deltaXY.angle()-PI/2; //angle would be PI/2 if we should point in y direction, but to the gimbal subsystem 0 is pointing in the y direction
        targetPitch = Reticle::solveForPitch(deltaXY.magnitude(), cvTarget.z); //gimbal subsystem will clamp the pitch. If it gets clamped, maybe don't shoot?
        gimbal->setAngles(targetYaw, targetPitch);
        targeting = true;
        bool shoot = cvTarget.booleans & 1; //from struct CVTarget in JetsonSubsystem.hpp
        if(allowShooting && shoot && startShotTimeout.execute()){
            indexer->tryShootOnce();
        }
    } else {
        //Haven't found a target, patrol
        targeting = false;

        numCyclesForBurst++;

        if(allowGimbal) {
            // getAngleToTurnForSentry() returns HitRing::PLACEHOLDER_ANGLE except on the single
            // cycle right after a hit is registered, when it returns (headYaw - hitDirection) in
            // world radians. Latch that one-shot value into an absolute world-yaw target and hold
            // it, otherwise it is lost the instant patrol resumes and the turret never turns.
            float angleToTurnForSentry = jetson->getAngleToTurnForSentry();
            bool turnToHit = cvTarget.booleans & 4; //from struct CVTarget in JetsonSubsystem.hpp
            if ((angleToTurnForSentry != HitRing::PLACEHOLDER_ANGLE) && angleToTurnForSentry) {
                targetPitch = 0.05;  // pitch down to avoid looking into the sky
                // Face the hit: target heading = current heading minus the returned offset.
                hitTargetYaw = gimbal->getYawAngleRelativeWorld() - angleToTurnForSentry;
                turningToHit = true;
                hitTurnStartTime = tap::arch::clock::getTimeMilliseconds();
            }

            if (turningToHit && tap::arch::clock::getTimeMilliseconds() - hitTurnStartTime < HIT_TURN_DURATION) {
                // Hold the heading toward the hit. CV still runs at the top of execute(), so if the
                // attacker comes into view the shoot branch takes over and engages it.
                gimbal->setAngles(hitTargetYaw, targetPitch);
            } else {
                turningToHit = false;
                bool typeCBasedPatrol = cvTarget.booleans & 2; //from struct CVTarget in JetsonSubsystem.hpp
                if(typeCBasedPatrol) targetPitch = 0.05;  // pitch down to avoid looking into the sky
                
                float yawChange = 0;
                if (numCyclesForBurst == CYCLES_UNTIL_BURST) {
                    if(typeCBasedPatrol) yawChange = BURST_AMOUNT;
                    numCyclesForBurst = 0;
                } else {
                    if(typeCBasedPatrol) yawChange = PATROL_SPEED;
                }
                gimbal->updateMotors(yawChange, targetPitch);
            }
        }
    }

    if(!allowShooting){
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
    isScheduled = false;
}

bool AutoAimAndFireCommand::getIsScheduled() { return isScheduled; }


bool AutoAimAndFireCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands
