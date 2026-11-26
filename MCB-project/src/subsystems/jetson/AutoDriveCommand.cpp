#include "AutoDriveCommand.hpp"

#include <algorithm>
#include <cmath>

#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"

namespace commands {
using tap::communication::serial::RefSerialData;

void AutoDriveCommand::initialize() {
    isScheduled = true;
    hasNavGoal = false;
    spinning = false;
    travelAligned = false;
    previousGoal.reset();
    corner.reset();
    drivetrain->resetPositionIntegral();
}

void AutoDriveCommand::execute() {
    const Vector2d current(odo->getX(), odo->getY());
    Vector2d goal, goalVelocity, expectedPosition;
    if (jetson->updateROS(&goal, &goalVelocity, &expectedPosition)) {
        if (!hasNavGoal || !(goal == targetPosition)) {
            const bool reversing = previousGoal && goal == *previousGoal;
            // Capture the outgoing leg near the corner before targeting its end.
            if (hasNavGoal && !spinning && !reversing &&
                (targetPosition - current).magnitude() <= WAYPOINT_HANDOFF_RADIUS) {
                corner = targetPosition;
            } else {
                corner.reset();
            }
            if (hasNavGoal) previousGoal = targetPosition;
            spinning = false;
            drivetrain->resetPositionIntegral();
        }
        targetPosition = goal;
        hasNavGoal = true;
    }
    if (!odo->isNavigationLocalized() || !hasNavGoal) {
        travelAligned = false;
        drivetrain->stopMotors();
        return;
    }

    // Reference angle is minus the field heading. The IMU is on the gimbal.
    const float referenceAngle = gimbal->getYawEncoderValue() -
                                 gimbal->getYawAngleRelativeWorld() - odo->getStartYaw();
    Pose2d currentPosition(current.getX(), current.getY(), referenceAngle);
    const auto &game = drivers->refSerial.getGameData();
    using GameStage = RefSerialData::Rx::GameStage;
    if (drivers->refSerial.getRefSerialReceivingData() &&
        game.gameType == RefSerialData::Rx::GameType::ROBOMASTER_RMUL_3V3 &&
        game.gameStage != GameStage::IN_GAME) {
        spinning = false;
        travelAligned = false;
        drivetrain->resetPositionIntegral();
        const float yaw = game.gameStage == GameStage::COUNTDOWN ? SPIN_VELOCITY : 0.0f;
        drivetrain->setTargetPosition(currentPosition.vec(), currentPosition, Pose2d(0, 0, yaw));
        return;
    }

    const float headingError = std::remainder(referenceAngle, PI / 2);
    const float yawRate = drivers->bmi088.getGz() - gimbal->getYawVel();
    const bool squaredUp = std::abs(headingError) <= TRAVEL_HEADING_ERROR;
    const bool settled = squaredUp && std::abs(yawRate) <= SETTLED_YAW_RATE;
    travelAligned = settled ||
                    (travelAligned && squaredUp && std::abs(yawRate) <= TRAVEL_YAW_RATE);

    Vector2d driveTarget = targetPosition;
    if (corner) {
        const Vector2d leg = targetPosition - *corner;
        const float length = leg.magnitude();
        const Vector2d direction = leg * (1.0f / length);
        const Vector2d offset = current - *corner;
        const float progress = offset.getX() * direction.getX() + offset.getY() * direction.getY();
        // Start with a nearby point on the new centreline. Lead grows with
        // progress up the leg, so the target reaches the endpoint continuously.
        const float targetProgress = CORNER_LOOKAHEAD + 2.0f * std::max(0.0f, progress);
        if (targetProgress < length) {
            driveTarget = *corner + direction * targetProgress;
        } else {
            corner.reset();
        }
    }
    const float distance = (targetPosition - current).magnitude();
    if (spinning) {
        spinning = distance <= SPIN_STOP_RADIUS;
    } else {
        spinning = settled && !corner && distance <= SPIN_START_RADIUS &&
                   std::hypot(odo->getXVel(), odo->getYVel()) <= SETTLED_SPEED;
    }

    const float yaw = spinning ? SPIN_VELOCITY : drivetrain->calculateRotationPID(headingError);
    if (spinning || !travelAligned) {
        travelAligned = false;
        drivetrain->resetPositionIntegral();
        drivetrain->setTargetPosition(currentPosition.vec(), currentPosition, Pose2d(0, 0, yaw));
    } else {
        drivetrain->setTargetPosition(driveTarget, currentPosition, Pose2d(0, 0, yaw));
    }
}

bool AutoDriveCommand::isFinished() const { return !drivers->remote.isConnected(); }

bool AutoDriveCommand::getIsScheduled() { return isScheduled; }

void AutoDriveCommand::end(bool) { isScheduled = false; }

}  // namespace commands
