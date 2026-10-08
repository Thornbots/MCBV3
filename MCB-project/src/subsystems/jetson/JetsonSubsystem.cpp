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
                odo->getX(),
                odo->getY(),
                odo->getXVel(),
                odo->getYVel(),
                gimbal->getPitchEncoderValue(),
                std::fmod(gimbal->getYawAngleRelativeWorld() + odo->getStartYaw(), 2 * PI),  // field yaw
                OdomStatus::ODOM_PODS
            };
            sendMsg(&p);
        } else {  // if(drivers->refSerial.getRefSerialReceivingData()) {

            tap::communication::serial::RefSerial::Rx::GameData gameData = drivers->refSerial.getGameData();
            tap::communication::serial::RefSerial::Rx::RobotData robotData = drivers->refSerial.getRobotData();
            angleToTurnForSentry = drivers->hitTracker.getAngleToTurnForSentry();
            RefSys r{
                (uint8_t)gameData.gameStage,
                (uint16_t)gameData.stageTimeRemaining,
                (uint16_t)robotData.currentHp,
                static_cast<uint8_t>(static_cast<uint8_t>(robotData.robotId) % 100),  // blue hero is 101, we want to send 1
                angleToTurnForSentry,
                // 12.34,
                static_cast<uint8_t>(drivers->refSerial.isBlueTeam(robotData.robotId) << 7 | (robotData.robotBuffStatus.recoveryBuff > 0) << 6 |
                    (robotData.rfidStatus.any(
                        tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::RESUPPLY_ZONE_OUTSIDE_EXCHANGE | tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::RESUPPLY_ZONE_INSIDE_EXCHANGE))
                        << 5 |
                    robotData.rfidStatus.any(tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::CENTRAL_BUFF) << 4 |
                    gameData.eventData.siteData.any(tap::communication::serial::RefSerial::Rx::SiteData::CENTRAL_BUFF_OCCUPIED_TEAM) << 3 |
                    gameData.eventData.siteData.any(tap::communication::serial::RefSerial::Rx::SiteData::CENTRAL_BUFF_OCCUPIED_OPPONENT) << 2 |
                    robotData.robotPower.any(tap::communication::serial::RefSerial::Rx::RobotPower::CHASSIS_HAS_POWER) << 1 |
                    robotData.robotPower.any(tap::communication::serial::RefSerial::Rx::RobotPower::GIMBAL_HAS_POWER))};
            // needToSendRefData = !
            sendMsg(&r);
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
