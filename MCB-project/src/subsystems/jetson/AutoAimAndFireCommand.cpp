#include "AutoAimAndFireCommand.hpp"
#include "JetsonSubsystemConstants.hpp"
#include "subsystems/odometry/OdometrySubsystemConstants.hpp"

#include <algorithm>
#include <cmath>


namespace commands {
using namespace tap::communication::serial;

AutoAimAndFireCommand::AutoAimAndFireCommand(src::Drivers* drivers, GimbalSubsystem* gimbal, IndexerSubsystem* indexer, FlywheelSubsystem* flywheel, JetsonSubsystem* jetson, OdometrySubsystem* odo, AutoDriveCommand* adc, bool isManualControl)
    : drivers(drivers),
        gimbal(gimbal),
        indexer(indexer),
        flywheel(flywheel),
        jetson(jetson),
        odo(odo), //can be nullptr for robots without odometry subsystem
        adc(adc), //can be nullptr for non-sentries
        isManualControl(isManualControl)
{
    addSubsystemRequirement(gimbal);
    if(!isManualControl) {
        addSubsystemRequirement(indexer);
        addSubsystemRequirement(flywheel);
    }
    startShotTimeout.stop();
    cvTargetValidTimeout.stop();
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
    
    // A pending shot holds its target: a newer frame would cancel it (from newMain's cv tuning).
    CvTarget newCvTarget{};
    //newerHasPriority if there is a newer target, current isn't shooting, and newer is shooting
    bool newTarget = jetson->getCvTarget(&newCvTarget);
    bool newerHasPriority = newTarget && (cvTarget.flags & CV_TARGET_FLAG_FIRE)==0 && (newCvTarget.flags & CV_TARGET_FLAG_FIRE)>0; 
    if(newTarget&&(startShotTimeout.isStopped() || newerHasPriority)) { //if current is expired or newer has priority
        cvTarget=newCvTarget;
        receivedCvTargetEver = true;
        cvTargetValidTimeout.restart(TARGET_VALID_TIME);
        // Shorter than the physical indexing latency means request immediately.
        startShotTimeout.restart(std::max(0, static_cast<int>(cvTarget.delay_ms) - FIRING_LATENCY_TIME));
    }
    // Due once, then released whether or not this cycle can fire, so targets keep flowing.
    const bool shotDue = startShotTimeout.execute();
    if(shotDue) startShotTimeout.stop();
    if(cvTargetValidTimeout.isExpired()) cvTargetValidTimeout.stop();
    float angleToTurnForSentry = drivers->hitTracker.getAngleToTurnForSentry();
    bool turnToHitFlag =        (cvTarget.flags & CV_TARGET_FLAG_TURN_TO_HIT)>0;
    bool typeCBasedPatrolFlag = (cvTarget.flags & CV_TARGET_FLAG_TYPE_C_BASED_PATROL)>0;
    bool shootFlag =            (cvTarget.flags & CV_TARGET_FLAG_FIRE)>0;
    bool needToTurnToHit = turnToHitFlag && drivers->hitTracker.didHitThisControlCycle();
    turningToHit &= turnToHitFlag;
    
    // set variables that ui debug can use
    targeting = allowGimbal&&!cvTargetValidTimeout.isStopped()&&!(needToTurnToHit||turningToHit);
    if(isManualControl) targeting &= shootFlag&&drivers->remote.getMouseR();
    // cvTarget and getAim* share the raw aim frame, never relocalized. The gimbal's world yaw is
    // the IMU's, counterclockwise and zero at power-on, so take the start's yaw off
    float odoAimX = 0;
    float odoAimY = 0;
    float odoStartYaw = odo::BOOT_YAW;
    if(odo!=nullptr){
        odoAimX = odo->getAimX();
        odoAimY = odo->getAimY();
        odoStartYaw = odo->getStartYaw();
    }
    Vector2d deltaXY{cvTarget.x - odoAimX, cvTarget.y - odoAimY};
    Vector3d deltasUnadjusted{deltaXY.getX(), deltaXY.getY(), cvTarget.z};
    Vector3d deltasAdjusted = Projections::robotSpaceToPivotSpace(deltasUnadjusted);
    deltaXY = {deltasAdjusted.getX(), deltasAdjusted.getY()};
    deltaX = deltasAdjusted.getX();
    deltaY = deltasAdjusted.getY();
    deltaZ = deltasAdjusted.getZ();
    // The barrel sits OFFSET_X_PITCH_PIVOT_TO_BARREL right of the yaw axis; turn so its line,
    // not the axis's, passes through the target.
    const float range = std::max(deltaXY.magnitude(), std::abs(Projections::OFFSET_X_PITCH_PIVOT_TO_BARREL) + 1e-3f);
    targetYaw = deltaXY.angle() + std::asin(Projections::OFFSET_X_PITCH_PIVOT_TO_BARREL / range) - odoStartYaw;
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
        if(shotDue && allowShooting && shootFlag){
            indexer->tryShootOnce();
        }
    } else {
        if(isManualControl){
            MouseMoveCommand::executeWith(drivers, gimbal);
        } else if (allowGimbal){ //automatic movement
            // getAngleToTurnForSentry() returns a world-frame hit bearing for one control cycle.
            // Preserve that bearing as an absolute yaw target while the turret turns toward it.
            if (needToTurnToHit) {
                // The tracker already expressed the hit bearing in the gimbal's world-yaw frame.
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
        if(allowShooting){
            indexer->idle();
        } else {
            indexer->stopIndex();
        }
        if(allowGimbal) {
            flywheel->setTargetVelocity(FLYWHEEL_MOTOR_MAX_RPM);
        } else {
            gimbal->stopMotors();
            if(adc!=nullptr && adc->getIsScheduled()) flywheel->setTargetVelocity(FLYWHEEL_MOTOR_MAX_RPM/4);
        }
    }
    
    // blink the green led to show that the jetson sent a message that said to shoot
    bool greenled = false;
    if(newTarget) greenled = (newCvTarget.flags & CV_TARGET_FLAG_FIRE)>0;
    drivers->leds.set(tap::gpio::Leds::Green, greenled);
}

void AutoAimAndFireCommand::end(bool) {
    isScheduled = false;
    startShotTimeout.stop();
    cvTargetValidTimeout.stop();
}

bool AutoAimAndFireCommand::getIsScheduled() { return isScheduled; }


bool AutoAimAndFireCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands
