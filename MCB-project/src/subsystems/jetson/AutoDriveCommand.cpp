#include "AutoDriveCommand.hpp"

#include <cmath>

#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"

namespace commands {
using tap::communication::serial::RefSerialData;

void AutoDriveCommand::initialize() {
    isScheduled = true;
    hasNavGoal = false;
    spinning = false;
    previousGoal.reset();
    arrivalTarget.reset();
    drivetrain->resetPositionIntegral();
}

void AutoDriveCommand::execute() {
    const Vector2d current(odo->getX(), odo->getY());
    Vector2d goal, goalVelocity, expectedPosition;
    if (jetson->updateROS(&goal, &goalVelocity, &expectedPosition)) {
        if (!hasNavGoal || !(goal == targetPosition)) {
            const bool reversing = previousGoal && goal == *previousGoal;
            // The publisher can step to the next leg while the chassis is still
            // braking outside the corner. Finish the corner before turning.
            if (hasNavGoal && !spinning && !reversing &&
                (targetPosition - current).magnitude() <= WAYPOINT_HANDOFF_RADIUS) {
                arrivalTarget = targetPosition;
            } else {
                arrivalTarget.reset();
            }
            if (hasNavGoal) previousGoal = targetPosition;
            spinning = false;
            drivetrain->resetPositionIntegral();
        }
        targetPosition = goal;
        hasNavGoal = true;
    }
    if (!odo->isNavigationLocalized() || !hasNavGoal) {
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
        drivetrain->resetPositionIntegral();
        const float yaw = game.gameStage == GameStage::COUNTDOWN ? SPIN_VELOCITY : 0.0f;
        drivetrain->setTargetPosition(currentPosition.vec(), currentPosition, Pose2d(0, 0, yaw));
        return;
    }

    const float headingError = std::remainder(referenceAngle, PI / 2);
    const float yawRate = drivers->bmi088.getGz() - gimbal->getYawVel();
    const bool aligned = std::abs(headingError) <= TRAVEL_HEADING_ERROR &&
                         std::abs(yawRate) <= TRAVEL_YAW_RATE;
    const auto stopped = [this] {
        return std::hypot(odo->getXVel(), odo->getYVel()) <= SETTLED_SPEED;
    };
    if (arrivalTarget && (*arrivalTarget - current).magnitude() <= WAYPOINT_SETTLED_RADIUS &&
        stopped()) {
        arrivalTarget.reset();
        drivetrain->resetPositionIntegral();
    }
    const float distance = (targetPosition - current).magnitude();
    if (spinning) {
        spinning = distance <= SPIN_STOP_RADIUS;
    } else {
        spinning = aligned && !arrivalTarget && distance <= SPIN_START_RADIUS && stopped();
    }

    const float yaw = spinning ? SPIN_VELOCITY : drivetrain->calculateRotationPID(headingError);
    if (spinning || !aligned) {
        drivetrain->resetPositionIntegral();
        drivetrain->setTargetPosition(currentPosition.vec(), currentPosition, Pose2d(0, 0, yaw));
    } else {
        drivetrain->setTargetPosition(arrivalTarget.value_or(targetPosition), currentPosition,
                                      Pose2d(0, 0, yaw));
    }
}

bool AutoDriveCommand::isFinished() const { return !drivers->remote.isConnected(); }

bool AutoDriveCommand::getIsScheduled() { return isScheduled; }

void AutoDriveCommand::end(bool) { isScheduled = false; }

}  // namespace commands
