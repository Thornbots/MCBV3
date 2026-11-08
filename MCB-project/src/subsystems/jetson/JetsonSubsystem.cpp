#include "JetsonSubsystem.hpp"

#include "JetsonSubsystemConstants.hpp"
#include "subsystems/drivetrain/SimpleAutoDriveCommand.hpp"

namespace subsystems {

JetsonSubsystem::JetsonSubsystem(src::Drivers* drivers, GimbalSubsystem* gimbal, OdometrySubsystem* odo) : tap::control::Subsystem(drivers), drivers(drivers), gimbal(gimbal), odo(odo) {}

void JetsonSubsystem::initialize() {
    drivers->commandScheduler.registerSubsystem(this);
    // comm.initialize();
}
int messageCount = 0;
void JetsonSubsystem::refresh() {
    if (odo != nullptr) checkApplyRelocalize();

    drivers->uart.updateSerial();

    hitRing.update();
    
    Ping ping{};
    if(getMsg(&ping)){
        sendMsg(&ping);
    }

    if (poseDataTimeout.execute()) {
        messageCount++;

        // 9 poses, 1 ref sys msg
        if (messageCount < 10 && odo != nullptr) {
            Pose p{
                odo->getAimX(),
                odo->getAimY(),
                odo->getXVel(),
                odo->getYVel(),
                gimbal->getPitchEncoderValue(),
                std::fmod(gimbal->getYawAngleRelativeWorld() + odo->getStartYaw(), 2 * PI),  // field yaw
#if defined(SENTRY)
                drivers->i2c.odom.isFresh(tap::arch::clock::getTimeMilliseconds(), POD_STALE_MS)
                    ? OdomStatus::ODOM_PODS : OdomStatus::ODOM_I2C_DEAD
#else
                OdomStatus::ODOM_PODS
#endif
            };
            sendMsg(&p);
        } else {  // if(drivers->refSerial.getRefSerialReceivingData()) {
            const float chassisWorldYaw = drivers->bmi088.getYaw() - gimbal->getYawEncoderValue();
            const float deltaAngleGotHitIn = drivers->hitTracker.getAngleForRefSys(chassisWorldYaw);
            RefSys r{
                (uint8_t)drivers->refWrapper.getGameStage(),
                (uint16_t)drivers->refWrapper.getStageTime(),
                drivers->refWrapper.getCurrentHp(),
                drivers->refWrapper.getRedTeamId(),  // blue hero is 101, we want to send 1
                deltaAngleGotHitIn,
                (uint8_t) (drivers->refWrapper.isBlue() << 7 | 
                drivers->refWrapper.isHealing() << 6 |
                drivers->refWrapper.isInHealZone() << 5 |
                drivers->refWrapper.isAtCenter() << 4 |
                drivers->refWrapper.teamHasCenter() << 3 |
                drivers->refWrapper.opponentHasCenter() << 2 |
                drivers->refWrapper.chassisHasPower() << 1 |
                drivers->refWrapper.gimbalHasPower())};
            // needToSendRefData = !
            if (sendMsg(&r)) drivers->hitTracker.consumeReportedHit();
            messageCount = 0;
        }
    }
}

void JetsonSubsystem::checkApplyRelocalize() {
    Relocalize relocalize_msg;
    if (getMsg(&relocalize_msg)) {
        odo->relocalizeTo(relocalize_msg.x, relocalize_msg.y);
    }
}

// TODO: ros-driven navigation is going likely going to change a lot
bool JetsonSubsystem::updateROS(Vector2d* targetPosition, Vector2d* targetVelocity, Vector2d*) {
    NavGoal navGoal;
    if (!getMsg(&navGoal)) return false;
    *targetPosition = Vector2d(navGoal.x, navGoal.y);
    *targetVelocity = Vector2d(0, 0);

    return true;
}

// Updates the contents of cvTarget and returns true if there was a new message, false if there wasn't a new message.
bool JetsonSubsystem::getCvTarget(CvTarget* cvTarget) {
    return getMsg(cvTarget);
}
};  // namespace subsystems
