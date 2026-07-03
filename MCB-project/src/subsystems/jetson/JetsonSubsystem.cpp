#include "JetsonSubsystem.hpp"

#include "JetsonSubsystemConstants.hpp"

float posXdebug;
float posYdebug;
float posZdebug;
float posXrelC;
float posYrelC;
float posZrelC;
float yawouttest;
float yawtest2;
float currentYawTest;
float currentPitchTest;
float velXdebug;
float velYdebug;
float velZdebug;
float targetPitchTest;

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

    // Push the current turret orientation into the delay line every cycle so that a latent CV
    // frame can be paired with the orientation from when it was actually captured.
    recordOrientationSample();

    if (poseDataTimeout.execute()) {
        messageCount++;

        // 9 poses, 1 ref sys msg
        if (messageCount < 10 && odo != nullptr) {
            PoseData p{
                odo->getX(),
                odo->getY(),
                odo->getXVel(),
                odo->getYVel(),
                gimbal->getPitchEncoderValue(),
                gimbal->getYawAngleRelativeWorld(),
                // drivers->bmi088.getq0(),
                // drivers->bmi088.getq1(),
                // drivers->bmi088.getq2(),
                // drivers->bmi088.getq3(),
                // drivers->bmi088.getAx(),
                // drivers->bmi088.getAy(),
                // drivers->bmi088.getAz()
            };
            sendMsg(&p);
        } else {  // if(drivers->refSerial.getRefSerialReceivingData()) {

            tap::communication::serial::RefSerial::Rx::GameData gameData = drivers->refSerial.getGameData();
            tap::communication::serial::RefSerial::Rx::RobotData robotData = drivers->refSerial.getRobotData();

            RefSysMsg r{
                (uint8_t)gameData.gameStage,
                (uint16_t)gameData.stageTimeRemaining,
                (uint16_t)robotData.currentHp,
                (uint8_t)robotData.robotId % 100,  // blue hero is 101, we want to send 1
                hitRing.getAngleToTurnForSentry(),
                // 12.34,
                drivers->refSerial.isBlueTeam(robotData.robotId) << 7 | (robotData.robotBuffStatus.recoveryBuff > 0) << 6 |
                    robotData.rfidStatus.all(tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::RESUPPLY_ZONE_OUTSIDE_EXCHANGE) << 5 |
                    robotData.rfidStatus.all(tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::CENTRAL_BUFF) << 4 |
                    gameData.eventData.siteData.all(tap::communication::serial::RefSerial::Rx::SiteData::CENTRAL_BUFF_OCCUPIED_OWN) << 3 |
                    gameData.eventData.siteData.all(tap::communication::serial::RefSerial::Rx::SiteData::CENTRAL_BUFF_OCCUPIED_OPPONENT) << 2 |
                    robotData.robotPower.all(tap::communication::serial::RefSerial::Rx::RobotPower::CHASSIS_HAS_POWER) << 1 |
                    robotData.robotPower.all(tap::communication::serial::RefSerial::Rx::RobotPower::GIMBAL_HAS_POWER)};
            // needToSendRefData = !
            sendMsg(&r);
            messageCount = 0;
        }
    }
}

void JetsonSubsystem::recordOrientationSample() {
    // world-frame turret yaw straight from the IMU quaternion (same convention as update()).
    float sq0 = drivers->bmi088.getq0();
    float sq1 = -drivers->bmi088.getq1();  // negated to swap IMU frame -> 3d-dynamics frame
    float sq2 = -drivers->bmi088.getq2();
    float sq3 = drivers->bmi088.getq3();

    OrientationSample sample;
    sample.cvYaw = atan2f(sq0 * sq3 - sq1 * sq2, -0.5f + (sq1 * sq1 + sq0 * sq0));  // XYZ 3rd rotation
    sample.cvYawVel = drivers->bmi088.getGz() * PI / 180;                           // body-z rate, rad/s

    // Pitch comes from the gimbal (the source update() previously used via current_pitch), negated
    // to match the transform's frame of reference exactly as `cvPitch = -current_pitch` did.
    sample.cvPitch = -gimbal->getPitchEncoderValue();
    sample.cvPitchVel = -gimbal->getPitchVel();

    orientationQueue[orientationQueueHead] = sample;
    orientationQueueHead = (orientationQueueHead + 1) % ORIENTATION_QUEUE_SIZE;
}

const OrientationSample& JetsonSubsystem::getDelayedOrientation() const {
    // The oldest entry in the ring (the slot about to be overwritten) is the orientation from
    // ~ORIENTATION_QUEUE_SIZE cycles ago.
    return orientationQueue[orientationQueueHead];
}

void JetsonSubsystem::checkApplyRelocalize() {
    Relocalize relocalize_msg;
    if (getMsg(&relocalize_msg)) {
        odo->relocalizeTo(relocalize_msg.expectedX, relocalize_msg.expectedY);
    }
}

bool JetsonSubsystem::updateROS(Vector2d* targetPosition, Vector2d* targetVelocity, Vector2d* jetsonExpectedPosition) {
    Relocalize relocalize_msg;
    if (getMsg(&relocalize_msg)) {
        *jetsonExpectedPosition = Vector2d(relocalize_msg.expectedX, relocalize_msg.expectedY);
        // x is 5, y is 3
        if (relocalize_msg.expectedX > 4) drivers->leds.set(tap::gpio::Leds::Blue, true);
        if (relocalize_msg.expectedY > 2) drivers->leds.set(tap::gpio::Leds::Green, true);
    };

    ROSData ros_msg;
    if (!getMsg(&ros_msg)) return false;
    *targetPosition = Vector2d(ros_msg.targetX, ros_msg.targetY);
    *targetVelocity = Vector2d(0, 0);

    return true;
}

void JetsonSubsystem::update(
    float current_yaw,
    float current_pitch,
    float current_yaw_velo,
    float current_pitch_velo,
    float* yawOut,
    float* pitchOut,
    float* yawVelOut,
    float* pitchVelOut,
    int* action) {
    *action = -1;

    CVData cv_msg;
    if (!getMsg(&cv_msg)) return;
    CVData* msg = &cv_msg;
    
    // white led on first cv message
    drivers->leds.set(tap::gpio::Leds::Red, true);
    drivers->leds.set(tap::gpio::Leds::Green, true);
    drivers->leds.set(tap::gpio::Leds::Blue, true);

    // Add rotated offset vector of panel relative to RGB
    if (msg->confidence <= MSG_CONFIDENCE_CUTOFF) return;

    // float X_prime = -x + 0.0175;                                                     // left
    // float Y_prime = -y + 0.1295 * cos(current_pitch) - 0.0867 * sin(current_pitch);  // up
    // float Z_prime = z + 0.0867 * cos(current_pitch) + 0.1295 * sin(current_pitch);   // forwards

    // TODO: just store vec3s in JetosnComms msg struct
    //  modm::Vector3f pos(msg->x,msg->y,msg->z);
    //  modm::Vector3f vel(msg->v_x,msg->v_y,msg->v_z);
    //  modm::Vector3f acc(msg->a_x,msg->a_y,msg->a_z);

    // MeasuredKinematicState state;//(pos,vel,acc);
    // state.position = modm::Vector3f(0,0,0);
    // state.velocity = modm::Vector3f(0,0,0);
    // state.acceleration = modm::Vector3f(0,0,0);

    // ===== CV latency compensation =====
    // This panel was captured ~ORIENTATION_QUEUE_SIZE control cycles ago (camera exposure + Jetson
    // processing + UART transport). Transform it into the world frame using the turret orientation
    // from that moment, pulled from the delay line, instead of the live orientation. (current_yaw
    // stays live below because the output is a delta from where the turret points right now.)
    const OrientationSample& delayed = getDelayedOrientation();
    cvYaw = delayed.cvYaw;
    cvPitch = delayed.cvPitch;
    cvYawVel = delayed.cvYawVel;
    cvPitchVel = delayed.cvPitchVel;

    currentYawTest = cvYaw;
    currentPitchTest = cvPitch;

    // get position of camera relative to shooting axis

    posXrel4 = msg->x + cameraXoffset;
    posYrel4 = msg->z + cameraYoffset;  // taproot flips z y basis vec
    posZrel4 = msg->y + cameraZoffset;   // msg->y is up; Z4 must point up so the pitch un-rotation
                                         // gives true world height (mz*sinφ + my*cosφ), not the
                                         // φ-dependent mz*sinφ - my*cosφ that caused the unstable
                                         // 2φ-E feedback / endstop slam
    velXrel4 = msg->v_x;
    velYrel4 = msg->v_z;
    velZrel4 = msg->v_y;
    // precompute commonly used angles


    // all of these values should be computed off of the queued values of cvYaw and cvPitch, not the current values of yaw and pitch,
    float cos_theta3 = cosf(cvYaw);
    float sin_theta3 = sinf(cvYaw);
    float cos_theta4 = cosf(cvPitch);
    float sin_theta4 = sinf(cvPitch);

    // convert camera offset into reference frame 2
    posXrelPitch = cos_theta3 * posXrel4 + sin_theta3 * (-cos_theta4 * posYrel4 + sin_theta4 * posZrel4);
    posYrelPitch = sin_theta3 * posXrel4 + cos_theta3 * (cos_theta4 * posYrel4 - sin_theta4 * posZrel4);
    posZrelPitch = sin_theta4 * posYrel4 + cos_theta4 * posZrel4;

    // get the velocity of the camera in reference frame 2
    velXrelPitch = (-sin_theta3 * posXrel4 - cos_theta3 * (cos_theta4 * posYrel4 - sin_theta4 * posZrel4)) * cvYawVel +
                   sin_theta3 * (sin_theta4 * posYrel4 + cos_theta4 * posZrel4) * cvPitchVel + cos_theta3 * velXrel4 - sin_theta3 * (cos_theta4 * velYrel4 - sin_theta4 * velZrel4);

    velYrelPitch = (cos_theta3 * posXrel4 - sin_theta3 * (cos_theta4 * posYrel4 - sin_theta4 * posZrel4)) * cvYawVel -
                   cos_theta3 * (sin_theta4 * posYrel4 + cos_theta4 * posZrel4) * cvPitchVel + sin_theta3 * velXrel4 + cos_theta3 * (cos_theta4 * velYrel4 - sin_theta4 * velZrel4);

    velZrelPitch = cvPitchVel  * cos_theta4 * posYrel4 + sin_theta4 * velYrel4 - cvPitchVel * sin_theta4 * posZrel4 + cos_theta4 * velZrel4;

    // get the velocity of the panel in 2

    modm::Vector3f position(posYrelPitch, -posXrelPitch, posZrelPitch);  // X is down range

    // velocity doesn't work well until the latency of the pipeline is significantly reduced
    modm::Vector3f velocity(velYrelPitch / 4.0f, -velXrelPitch / 4.0f, velZrelPitch / 100.0f);
    modm::Vector3f acceleration(msg->a_x, msg->a_z, -msg->a_y);  // not imlemented yet afak

    SecondOrderKinematicState state(position, velocity, acceleration);  //(pos,vel,acc);

    float targetYaw, targetPitch, travelTime;
    bool valid = tap::algorithms::ballistics::findTargetProjectileIntersection(state, initialShotVelocity, 3, &targetPitch, &targetYaw, &travelTime, 0);

    posXdebug = posXrelPitch;
    posYdebug = posYrelPitch;
    posZdebug = posZrelPitch;
    velXdebug = velXrelPitch;
    velYdebug = velYrelPitch;
    velZdebug = velZrelPitch;
    posXrelC = posXrel4;
    posZrelC = posZrel4;
    posYrelC = posYrel4;
    targetPitchTest = targetPitch;

    float sq0 = drivers->bmi088.getq0();
    float sq1 = -drivers->bmi088.getq1();  // negated to swap IMU frame -> 3d-dynamics frame
    float sq2 = -drivers->bmi088.getq2();
    float sq3 = drivers->bmi088.getq3();

    float current_yaw_2 = atan2f(sq0 * sq3 - sq1 * sq2, -0.5f + (sq1 * sq1 + sq0 * sq0));  // XYZ 3rd rotation
    yawouttest = (targetYaw - current_yaw_2); //target yaw is computed from the properly queued yaw, and it needs to be compared to the current yaw to properly adjust
    yawtest2 = targetYaw;

    if (!valid) {
        *action = -1;  // make enums for action
        return;
    }

    *yawOut = (targetYaw - current_yaw_2);  // fmod(current_yaw + targetYaw, 2 * PI);
    *pitchOut = targetPitch;
    *yawVelOut = (-cos_theta3 * velXrelPitch - sin_theta3 * velYrelPitch + velXrel4) / (cos_theta4 * posYrel4 - sin_theta4 * posZrel4);
    *pitchVelOut = 0; //unsure on how to compute this well and i dont trust AI

    if (abs(*yawOut) < YAW_OUT_SHOOT_THRESH) {
        // Enable shooting
        *action = 1;
        return;
    }
    *action = 0;
}
};  // namespace subsystems
