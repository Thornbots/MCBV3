#include "AutoAimAndFireCommand.hpp"
#include "JetsonSubsystemConstants.hpp"


namespace commands {
using namespace tap::communication::serial;

AutoAimAndFireCommand::AutoAimAndFireCommand(src::Drivers* drivers, GimbalSubsystem* gimbal, IndexerSubsystem* indexer, FlywheelSubsystem* flywheel, JetsonSubsystem* jetson, OdometrySubsystem* odo, AutoDriveCommand* adc, bool isManualControl)
    : drivers(drivers),
        gimbal(gimbal),
        indexer(indexer),
        flywheel(flywheel),
        jetson(jetson),
        odo(odo),
        adc(adc),
        isManualControl(isManualControl)
{
    addSubsystemRequirement(gimbal);
    if(!isManualControl) {
        addSubsystemRequirement(indexer);
        addSubsystemRequirement(flywheel);
    }
}

void AutoAimAndFireCommand::initialize() {
    isScheduled = true;
}
void AutoAimAndFireCommand::execute() {
    bool allowShooting = true;
    bool allowGimbal = true;

    // if automatic, check if we are in a game, and if we are before it starts, don't allow aiming or shooting
    if (!isManualControl && drivers->refSerial.getRefSerialReceivingData() && 
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
    if(jetson->getCvTarget(&cvTarget)) {
        receivedCvTargetEver = true;
        cvTargetValidTimeout.restart(TARGET_VALID_TIME);
        startShotTimeout.restart(cvTarget.delay_ms-FIRING_LATENCY_TIME);
    }
    if(cvTargetValidTimeout.isExpired()) cvTargetValidTimeout.stop();
    float angleToTurnForSentry = drivers->hitTracker.getAngleToTurnForSentry();
    bool turnToHitFlag =        (cvTarget.flags & CV_TARGET_FLAG_TURN_TO_HIT)>0;
    bool typeCBasedPatrolFlag = (cvTarget.flags & CV_TARGET_FLAG_TYPE_C_BASED_PATROL)>0;
    bool shootFlag =            (cvTarget.flags & CV_TARGET_FLAG_FIRE)>0;
    bool needToTurnToHit = turnToHitFlag && drivers->hitTracker.isHit;
    turningToHit &= turnToHitFlag;
    
    // set variables that ui debug can use
    targeting = allowGimbal&&!cvTargetValidTimeout.isStopped()&&!(needToTurnToHit||turningToHit);
    if(isManualControl) targeting &= shootFlag&&drivers->remote.getMouseR();
    // cvTarget and the odometry are both in the field frame. The gimbal's world yaw is
    // the IMU's, counterclockwise and zero at power-on, so take the start's yaw off
    Vector2d deltaXY{cvTarget.x - odo->getX(), cvTarget.y - odo->getY()};
    deltaX = deltaXY.getX();
    deltaY = deltaXY.getY();
    deltaZ = cvTarget.z-Projections::OFFSET_Z_ROBOT_TO_PITCH_PIVOT;
    targetYaw = deltaXY.angle() - odo->getStartYaw();
    targetPitch = Reticle::solveForPitch(deltaXY.magnitude(), deltaZ); //gimbal subsystem will clamp the pitch. If it gets clamped, maybe don't shoot?
    
    if (targeting) { //do position-based aiming
        // the tap::algorithms::ballistics::findTargetProjectileIntersection function
        // is useful for knowing how to hit a moving target, but the jetson already
        // did that work. So we just need to do simple projectile motion to aim
        // at a position that isn't moving.
        // The cvTarget xyz is in the field frame, so if odo thinks we are at (3, 4)
        // and cvTarget says to aim at (5, 4, 0.2), we need to aim toward +x. We adjust
        // our aiming to hit the cvTarget as odo moves, so if we move to (3, 3.8)
        // while we are aiming, we need to look slightly to the left of +x.

        // Note that for gimbal subsystem, positive pitch is downward.
        gimbal->setAngles(targetYaw, targetPitch);
        if(startShotTimeout.execute() && allowShooting && shootFlag){
            indexer->tryShootOnce();
        }
    } else {
        if(isManualControl){
            MouseMoveCommand::executeWith(drivers, gimbal);
        } else if (allowGimbal){ //automatic movement
            // getAngleToTurnForSentry() returns HitRing::PLACEHOLDER_ANGLE except on the single
            // cycle right after a hit is registered, when it returns (headYaw - hitDirection) in
            // world radians. Latch that one-shot value into an absolute world-yaw target and hold
            // it, otherwise it is lost the instant patrol resumes and the turret never turns.
            if (needToTurnToHit) {
                // Face the hit: target heading = current heading minus the returned offset.
                hitTargetYaw = angleToTurnForSentry;
                turningToHit = true;
                hitTurnStartTime = tap::arch::clock::getTimeMilliseconds();
            }

            if (turningToHit && tap::arch::clock::getTimeMilliseconds() - hitTurnStartTime < HIT_TURN_DURATION) {
                // Hold the heading toward the hit. CV still runs at the top of execute(), so if the
                // attacker comes into view the shoot branch takes over and engages it.
                gimbal->setAngles(hitTargetYaw, PATROL_PITCH);
            } else {
                turningToHit = false;
                float yawChange = 0;
                if(typeCBasedPatrolFlag) {
                    numCyclesForBurst++;
                    if (numCyclesForBurst == CYCLES_UNTIL_BURST) {
                        yawChange = BURST_AMOUNT;
                        numCyclesForBurst = 0;
                    } else {
                        yawChange = PATROL_SPEED;
                    }
                }
                gimbal->updateMotors(yawChange, PATROL_PITCH);
            }
        }
    }

    if(!isManualControl){
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
}

void AutoAimAndFireCommand::end(bool) {
    isScheduled = false;
}

bool AutoAimAndFireCommand::getIsScheduled() { return isScheduled; }


bool AutoAimAndFireCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands
