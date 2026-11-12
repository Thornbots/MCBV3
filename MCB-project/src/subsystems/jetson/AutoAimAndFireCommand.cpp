#include "AutoAimAndFireCommand.hpp"
#include "JetsonSubsystemConstants.hpp"
#include "AimTiming.hpp"
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
    cvTargetValidTimeout.stop();
}

aim::AimTuning AutoAimAndFireCommand::makeTuning() {
    aim::AimTuning t;
    t.firmware_latency_s = FIRING_LATENCY_TIME * 0.001f;
    t.v_muzzle_mps = initialShotVelocity;
    t.ballistics = Projections::ballisticModel(initialShotVelocity);
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
    in.clear = (s.flags & TARGET_STATE_FLAG_CLEAR) != 0;
    return in;
}
}  // namespace

void AutoAimAndFireCommand::initialize() {
    isScheduled = true;
    aim::enable_aim_timing();
    planner.reset();
    holdingTargetState = false;
    cvTargetValidTimeout.stop();
}
void AutoAimAndFireCommand::execute() {
    bool allowShooting = true;
    bool allowGimbal = true;

    if (!isManualControl && drivers->refSerial.getRefSerialReceivingData() &&
        drivers->refSerial.getGameData().gameType == RefSerialData::Rx::GameType::ROBOMASTER_RMUL_3V3) {
        const auto stage = drivers->refSerial.getGameData().gameStage;
        allowShooting = stage == RefSerialData::Rx::GameStage::IN_GAME;
        allowGimbal = allowShooting || stage == RefSerialData::Rx::GameStage::COUNTDOWN;
    }

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

    TargetState newState{};
    uint32_t receivedMs = 0;
    while (jetson->getTargetState(&newState, &receivedMs)) {
        if (!planner.observe(receivedMs, toInput(newState, 0))) continue;
        receivedCvTargetEver = true;
        turnToHitPermitted = false;
        patrolPermitted = (newState.flags & TARGET_STATE_FLAG_TYPE_C_BASED_PATROL) != 0;
    }

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

    const aim::Vec3 pivot{odoAimX, odoAimY, Projections::OFFSET_Z_ROBOT_TO_PITCH_PIVOT};
    const aim::Vec3 shooterVel{odoVelX, odoVelY, 0};
#ifdef MCB_AIM_TIMING
    const uint32_t planStart = aim::cycle_now();
#endif
    const aim::Plan plan = planner.tick(nowMs, pivot, shooterVel);
#ifdef MCB_AIM_TIMING
    const uint32_t planCycles = aim::cycle_now()-planStart;
    aim::aim_timing.control.record(planCycles);
    if (planner.replanned()) {
        aim::aim_timing.replan.record(planCycles);
        if (!plan.valid) aim::aim_timing.failed_plan.record(planCycles);
        if (plan.panel != timingPanel) aim::aim_timing.handoff.record(planCycles);
        timingPanel = plan.panel;
    }
#endif
    const bool haveTargetState = planner.state() == aim::AimControl::State::TRACKING;
    holdingTargetState = haveTargetState;

    float angleToTurnForSentry = drivers->hitTracker.getAngleToTurnForSentry();
    bool needToTurnToHit = !haveTargetState && turnToHitPermitted && drivers->hitTracker.didHitThisControlCycle();
    turningToHit &= turnToHitPermitted && !haveTargetState;

    // set variables that ui debug can use
    const bool haveAim = haveTargetState ? plan.valid : !cvTargetValidTimeout.isStopped();
    targeting = allowGimbal&&haveAim&&!(needToTurnToHit||turningToHit);
    if(isManualControl) targeting &= haveTargetState&&drivers->remote.getMouseR();
    // The aim point and getAim* share the raw aim frame, never relocalized. The gimbal's world yaw is
    // the IMU's, counterclockwise and zero at power-on, so take the start's yaw off
    const bool aimAtState = haveTargetState && plan.valid;
    const Vector3d aimWorld = aimAtState ? Vector3d{plan.aim.x, plan.aim.y, plan.aim.z} : Vector3d{cvTarget.x, cvTarget.y, cvTarget.z};
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
    targetPitch = aimAtState ? plan.pitch : Reticle::solveForPitch(deltaXY.magnitude(), deltaZ);
    if (aimAtState) {
        targetYaw = plan.yaw-odoStartYaw;
        targetPitch = plan.pitch;
    }
    targeting &= std::isfinite(targetYaw) && std::isfinite(targetPitch);

    if (targeting) { //do position-based aiming
        // The MCB planner already led the target; here we only need simple projectile
        // motion to a point. Positions are in the field frame, so we adjust as odo moves.

        // Note that for gimbal subsystem, positive pitch is downward.
        if (aimAtState) {
            gimbal->setAnglesWithRates(targetYaw, targetPitch, plan.yaw_rate, plan.pitch_rate);
        } else {
            gimbal->setAngles(targetYaw, targetPitch);
        }
        const aim::GimbalMotion measured{gimbal->getYawAngleRelativeWorld()+odoStartYaw,
            gimbal->getAimPitch(), gimbal->getWorldYawRate(), gimbal->getAimPitchRate()};
#ifdef MCB_AIM_TIMING
        const uint32_t gateStart = aim::cycle_now();
#endif
        const bool canFire = allowShooting && planner.can_fire(nowMs, pivot, shooterVel, measured);
#ifdef MCB_AIM_TIMING
        aim::aim_timing.gate.record(aim::cycle_now()-gateStart);
#endif
        if (canFire) {
            indexer->tryShootOnce();
            lastFireRequestMs = nowMs;
            everRequestedFire = true;
        }
    } else {
        if(isManualControl){
            MouseMoveCommand::executeWith(drivers, gimbal);
        } else if (allowGimbal && !haveTargetState){ //automatic movement
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
    
    // Green: tracking an assigned target.
    drivers->leds.set(tap::gpio::Leds::Green, targeting && holdingTargetState);
}

void AutoAimAndFireCommand::end(bool) {
    isScheduled = false;
    cvTargetValidTimeout.stop();
    holdingTargetState = false;
    planner.reset();
}

bool AutoAimAndFireCommand::getIsScheduled() { return isScheduled; }


bool AutoAimAndFireCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands
