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

    if (poseDataTimeout.execute()) {
        messageCount++;

        // 9 poses, 1 ref sys msg
        if (messageCount < 10 && odo != nullptr) {
            Pose p{
                odo->getX(),
                odo->getY(),
                odo->getXVel(),
                odo->getYVel(),
                gimbal->getPitchEncoderValue(),
                gimbal->getYawAngleRelativeWorld(),
                OdomStatus::ODOM_PODS
            };
            sendMsg(&p);
        } else {  // if(drivers->refSerial.getRefSerialReceivingData()) {

            tap::communication::serial::RefSerial::Rx::GameData gameData = drivers->refSerial.getGameData();
            tap::communication::serial::RefSerial::Rx::RobotData robotData = drivers->refSerial.getRobotData();
            angleToTurnForSentry = hitRing.getAngleToTurnForSentry();
            RefSys r{
                (uint8_t)gameData.gameStage,
                (uint16_t)gameData.stageTimeRemaining,
                (uint16_t)robotData.currentHp,
                (uint8_t)robotData.robotId % 100,  // blue hero is 101, we want to send 1
                angleToTurnForSentry,
                // 12.34,
                drivers->refSerial.isBlueTeam(robotData.robotId) << 7 | (robotData.robotBuffStatus.recoveryBuff > 0) << 6 |
                    (robotData.rfidStatus.any(
                        tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::RESTORATION_ZONE | tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::EXCHANGE_ZONE))
                        << 5 |
                    robotData.rfidStatus.any(tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::CENTRAL_BUFF) << 4 |
                    gameData.eventData.siteData.any(tap::communication::serial::RefSerial::Rx::SiteData::CENTRAL_BUFF_OCCUPIED_TEAM) << 3 |
                    gameData.eventData.siteData.any(tap::communication::serial::RefSerial::Rx::SiteData::CENTRAL_BUFF_OCCUPIED_OPPONENT) << 2 |
                    robotData.robotPower.any(tap::communication::serial::RefSerial::Rx::RobotPower::CHASSIS_HAS_POWER) << 1 |
                    robotData.robotPower.any(tap::communication::serial::RefSerial::Rx::RobotPower::GIMBAL_HAS_POWER)};
            // needToSendRefData = !
            sendMsg(&r);
            messageCount = 0;
        }
    }
}

float JetsonSubsystem::getAngleToTurnForSentry() {
    float r = angleToTurnForSentry;
    angleToTurnForSentry = HitRing::PLACEHOLDER_ANGLE;
    return r;
}

void JetsonSubsystem::checkApplyRelocalize() {
    Relocalize relocalize_msg;
    if (getMsg(&relocalize_msg)) {
        // odo->relocalizeTo(relocalize_msg.x, relocalize_msg.y);
        commands::SimpleAutoDriveCommand::xForLocalization = relocalize_msg.x;
        commands::SimpleAutoDriveCommand::yForLocalization = relocalize_msg.y;
        commands::SimpleAutoDriveCommand::setLocalization = true;
    }
}

bool JetsonSubsystem::updateROS(Vector2d* targetPosition, Vector2d* targetVelocity, Vector2d* jetsonExpectedPosition) {
    Relocalize relocalize_msg;
    if (getMsg(&relocalize_msg)) {
        *jetsonExpectedPosition = Vector2d(relocalize_msg.x, relocalize_msg.y);
        // x is 5, y is 3
        if (relocalize_msg.x > 4) drivers->leds.set(tap::gpio::Leds::Blue, true);
        if (relocalize_msg.y > 2) drivers->leds.set(tap::gpio::Leds::Green, true);
    };

    NavGoal nav_goal;
    if (!getMsg(&nav_goal)) return false;
    *targetPosition = Vector2d(nav_goal.x, nav_goal.y);
    *targetVelocity = Vector2d(0, 0);

    return true;
}

void JetsonSubsystem::update(
    float /*current_yaw*/,
    float /*current_pitch*/,
    float /*current_yaw_velo*/,
    float /*current_pitch_velo*/,
    float* /*yawOut*/,
    float* /*pitchOut*/,
    float* /*yawVelOut*/,
    float* /*pitchVelOut*/,
    int* action) {
    *action = -1;

    // Take the frame so it doesn't block the one-slot mailbox. Aiming at the odom
    // point and firing on CV_TARGET_FLAG_FIRE after delay_ms aren't written yet,
    // so this never aims or shoots and AutoAimAndFireCommand patrols. The old
    // camera-frame solve is in git history before the CvTarget change.
    CvTarget cv_target;
    getMsg(&cv_target);
}
};  // namespace subsystems
