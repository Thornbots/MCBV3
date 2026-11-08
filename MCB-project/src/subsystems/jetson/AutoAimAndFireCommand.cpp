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
        isManualControl(isManualControl),
        planner(makeTuning())
{
    addSubsystemRequirement(gimbal);
    if(!isManualControl) {
        addSubsystemRequirement(indexer);
        addSubsystemRequirement(flywheel);
    }
    startShotTimeout.stop();
    cvTargetValidTimeout.stop();
}

aim::AimTuning AutoAimAndFireCommand::makeTuning() {
    aim::AimTuning t;
    t.firmware_latency_s = FIRING_LATENCY_TIME * 0.001f;
    t.v_muzzle_mps = initialShotVelocity;
    return t;
}

namespace {
aim::TargetInput toInput(const TargetState& s, float extraAgeS) {
    aim::TargetInput in;
    in.center = {s.center[0], s.center[1], s.center[2]};
    in.velocity = {s.velocity[0], s.velocity[1], s.velocity[2]};
    in.acceleration = {s.acceleration[0], s.acceleration[1], s.acceleration[2]};
    in.panel = {s.panel[0], s.panel[1], s.panel[2]};
    in.yaw = s.yaw;
    in.yaw_rate = s.yaw_rate;
    in.radius = {s.radius[0], s.radius[1]};
    in.z_offset = {s.z_offset[0], s.z_offset[1]};
    in.state_age_s = s.state_age_ms * 0.001f + extraAgeS;
    in.detection_age_s = s.detection_age_ms * 0.001f + extraAgeS;
    in.track_id = s.track_id;
    in.confidence = s.confidence / 255.0f;
    in.valid = (s.flags & TARGET_STATE_FLAG_VALID) != 0;
    return in;
}
}  // namespace

void AutoAimAndFireCommand::initialize() {
    isScheduled = true;
}
void AutoAimAndFireCommand::execute() {
    bool allowShooting = true;
    bool allowGimbal = true;

    // if automatic, check if we are in a game, and if we are before it starts, don't allow aiming or shooting
    // if (!isManualControl && drivers->refSerial.getRefSerialReceivingData() && 
    //    (drivers->refSerial.getGameData().gameType == RefSerialData::Rx::GameType::ROBOMASTER_RMUL_3V3)) {

    //     allowShooting = false;
    //     allowGimbal = false;

    //     if (drivers->refSerial.getGameData().gameStage == RefSerialData::Rx::GameStage::IN_GAME) {
    //         // allow both
    //         allowShooting = true;
    //         allowGimbal = true;
    //     }

    //     //don't spin before match
    //     if (drivers->refSerial.getGameData().gameStage == RefSerialData::Rx::GameStage::COUNTDOWN) {
    //         // countdown, only allow gimbal
    //         allowGimbal = true;
    //     }

    // }
    
    const uint32_t nowMs = tap::arch::clock::getTimeMilliseconds();

    // Idle look point: aimed at for its validity, never fired at.
    CvTarget newCvTarget{};
    if(jetson->getCvTarget(&newCvTarget)) {
        cvTarget = newCvTarget;
        receivedCvTargetEver = true;
        cvTargetValidTimeout.restart(TARGET_VALID_TIME);
        turnToHitPermitted = (cvTarget.flags & CV_TARGET_FLAG_TURN_TO_HIT)>0;
        patrolPermitted = (cvTarget.flags & CV_TARGET_FLAG_TYPE_C_BASED_PATROL)>0;
    }
    if(cvTargetValidTimeout.isExpired()) cvTargetValidTimeout.stop();

    // Tracked target state: held until its state age reaches the timeout.
    TargetState newState{};
    uint32_t receivedMs = 0;
    if(jetson->getTargetState(&newState, &receivedMs)) {
        heldState = newState;
        heldReceivedMs = receivedMs;
        haveTargetState = true;
        receivedCvTargetEver = true;
        turnToHitPermitted = false;  // nothing permits a hit turn until the next idle frame
        patrolPermitted = (newState.flags & TARGET_STATE_FLAG_TYPE_C_BASED_PATROL)>0;
        const float sinceReceipt = static_cast<int32_t>(nowMs - receivedMs) * 0.001f;
        planner.observe(nowMs, toInput(heldState, std::max<float>(sinceReceipt, 0.0f)));
    }
    float sinceReceiptS = 0;
    if(haveTargetState) {
        sinceReceiptS = std::max<int32_t>(static_cast<int32_t>(nowMs - heldReceivedMs), 0) * 0.001f;
        if(heldState.state_age_ms * 0.001f + sinceReceiptS >= planner.tuning().target_timeout_s) {
            haveTargetState = false;
            planner.reset();
        }
    }
    holdingTargetState = haveTargetState;

    // Due once, then released whether or not this cycle can fire, so targets keep flowing.
    const bool shotDue = startShotTimeout.execute();
    if(shotDue) startShotTimeout.stop();
    const bool shotPending = !startShotTimeout.isStopped();

    // odo can be nullptr for robots without odometry
    float odoAimX = 0;
    float odoAimY = 0;
    float odoVelX = 0;
    float odoVelY = 0;
    float odoStartYaw = odo::BOOT_YAW;
    if(odo!=nullptr){
        odoAimX = odo->getAimX();
        odoAimY = odo->getAimY();
        odoVelX = odo->getXVel();
        odoVelY = odo->getYVel();
        odoStartYaw = odo->getStartYaw();
    }

    // Replan each period; a pending shot holds the aim it was planned for.
    if(haveTargetState && !shotPending && static_cast<int32_t>(nowMs - nextReplanMs) >= 0) {
        nextReplanMs = nowMs + static_cast<uint32_t>(planner.tuning().replan_period_s * 1000.0f + 0.5f);
        const aim::TargetInput in = toInput(heldState, sinceReceiptS);
        // The planner measures time from the state's stamp, so the shooter starts where it was then.
        const aim::Vec3 shooterVel{odoVelX, odoVelY, 0.0f};
        const aim::Vec3 shooterPos = aim::Vec3{odoAimX, odoAimY, Projections::OFFSET_Z_ROBOT_TO_PITCH_PIVOT} - shooterVel * in.state_age_s;
        const aim::Plan plan = planner.plan(nowMs, in, shooterPos, shooterVel);
        heldAim = plan.aim;
        if(plan.fire) {
            startShotTimeout.restart(plan.request_delay_ms);
            lastFireRequestMs = nowMs;
            everRequestedFire = true;
        }
    }

    float angleToTurnForSentry = drivers->hitTracker.getAngleToTurnForSentry();
    bool needToTurnToHit = !haveTargetState && turnToHitPermitted && drivers->hitTracker.didHitThisControlCycle();
    turningToHit &= turnToHitPermitted && !haveTargetState;

    // set variables that ui debug can use
    const bool haveAim = haveTargetState || shotPending || !cvTargetValidTimeout.isStopped();
    targeting = allowGimbal&&haveAim&&!(needToTurnToHit||turningToHit);
    if(isManualControl) targeting &= haveTargetState&&drivers->remote.getMouseR();
    // The aim point and getAim* share the raw aim frame, never relocalized. The gimbal's world yaw is
    // the IMU's, counterclockwise and zero at power-on, so take the start's yaw off
    const bool aimAtState = haveTargetState || shotPending;
    const Vector3d aimWorld = aimAtState ? Vector3d{heldAim.x, heldAim.y, heldAim.z} : Vector3d{cvTarget.x, cvTarget.y, cvTarget.z};
    Vector2d deltaXY{aimWorld.getX() - odoAimX, aimWorld.getY() - odoAimY};
    Vector3d deltasUnadjusted{deltaXY.getX(), deltaXY.getY(), aimWorld.getZ()};
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
        // The MCB planner already led the target; here we only need simple projectile
        // motion to a point. Positions are in the field frame, so we adjust as odo moves.

        // Note that for gimbal subsystem, positive pitch is downward.
        gimbal->setAngles(targetYaw, targetPitch);
        if(shotDue && allowShooting){
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
                // Hold the heading toward the hit. A target state arriving ends the turn
                // (turningToHit is cleared above), so the aim branch takes over.
                gimbal->setAngles(hitTargetYaw, PATROL_PITCH);
            } else {
                turningToHit = false;
                float yawChange = 0;
                if(patrolPermitted) {
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
    
    // green led: a shot is scheduled
    drivers->leds.set(tap::gpio::Leds::Green, !startShotTimeout.isStopped());
}

void AutoAimAndFireCommand::end(bool) {
    isScheduled = false;
    startShotTimeout.stop();
    cvTargetValidTimeout.stop();
    haveTargetState = false;
    holdingTargetState = false;
    planner.reset();
}

bool AutoAimAndFireCommand::getIsScheduled() { return isScheduled; }


bool AutoAimAndFireCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands
