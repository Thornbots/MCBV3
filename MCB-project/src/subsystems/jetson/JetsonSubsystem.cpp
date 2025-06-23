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

JetsonSubsystem::JetsonSubsystem(src::Drivers* drivers, GimbalSubsystem* gimbal) :
     tap::control::Subsystem(drivers), drivers(drivers), gimbal(gimbal) {}


void JetsonSubsystem::initialize() {
    drivers->commandScheduler.registerSubsystem(this);
    // comm.initialize();
}

void JetsonSubsystem::refresh() { 
    drivers->uart.updateSerial(); 

    hitRing.update();

    //if we need to not lose a ref message, use needToSendRefData
    if(refDataSendingTimeout.execute() && drivers->refSerial.getRefSerialReceivingData()){
    //     needToSendRefData = true;
    // }

    // if(needToSendRefData){
        tap::communication::serial::RefSerial::Rx::GameData gameData = drivers->refSerial.getGameData();
        tap::communication::serial::RefSerial::Rx::RobotData robotData = drivers->refSerial.getRobotData();
        RefSysMsg r{
            (uint8_t) gameData.gameStage,
            (uint16_t) gameData.stageTimeRemaining,
            (uint16_t) robotData.currentHp,
            (uint8_t) robotData.robotId % 100, //blue hero is 101, we want to send 1
            hitRing.getAngleToTurnForSentry(), 
            // 12.34,

            drivers->refSerial.isBlueTeam(robotData.robotId) << 7 |
            (robotData.robotBuffStatus.recoveryBuff > 0) << 6 |
            (robotData.rfidStatus.any(tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::RESTORATION_ZONE | tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::EXCHANGE_ZONE)) << 5 |
            robotData.rfidStatus.any(tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::CENTRAL_BUFF) << 4 |
            gameData.eventData.siteData.any(tap::communication::serial::RefSerial::Rx::SiteData::CENTRAL_BUFF_OCCUPIED_TEAM) << 3 |
            gameData.eventData.siteData.any(tap::communication::serial::RefSerial::Rx::SiteData::CENTRAL_BUFF_OCCUPIED_OPPONENT) << 2 |
            robotData.robotPower.any(tap::communication::serial::RefSerial::Rx::RobotPower::CHASSIS_HAS_POWER) << 1 |
            robotData.robotPower.any(tap::communication::serial::RefSerial::Rx::RobotPower::GIMBAL_HAS_POWER) 
        };
        //needToSendRefData = !
        sendMsg(&r);
    }

    PoseData p{
        drivers->i2c.odom.getX(), //OdometrySubsystem doesn't provide wrappers for these
        drivers->i2c.odom.getY(),
        drivers->i2c.odom.getXVel(),
        drivers->i2c.odom.getYVel(),
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

    
}

bool JetsonSubsystem::updateROS(Vector2d* targetPosition, Vector2d* targetVelocity, Vector2d* jetsonExpectedPosition) {
    ROSData ros_msg;
    if(!getMsg(&ros_msg))
        return false;
    *targetPosition = Vector2d(ros_msg.targetX, ros_msg.targetY);
    *jetsonExpectedPosition = Vector2d(ros_msg.expectedX, ros_msg.expectedY);
    //todo make this work lmao
    *targetVelocity = Vector2d(0, 0);
    return true;
}

void JetsonSubsystem::update(float current_yaw, float current_pitch, float current_yaw_velo, float current_pitch_velo, float* yawOut, float* pitchOut, float* yawVelOut, float* pitchVelOut, int* action) {
    *action = -1;

    CVData cv_msg;
    if(!getMsg(&cv_msg))
        return;
    CVData* msg = &cv_msg;

    // Add rotated offset vector of panel relative to RGB
    // if (msg->confidence <= 0.2f) return;

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

    // get the quaternion directly
    q0 = drivers->bmi088.getq0();
    q1 = -drivers->bmi088.getq1();  // axis is negated to swap IMU reference frame to the 3d dynamics reference frame
    q2 = -drivers->bmi088.getq2();  // axis is negated to swap IMU reference frame to the 3d dynamics reference frame
    q3 = drivers->bmi088.getq3();

    // express the IMU orientation in XYZ euler angles
    //formula linked from matlab code
    //cvRoll =  // 1st rotation
    //cvPitch = asinf(std::clamp(2.0f * (q0 * q2 + q3 * q1), -1.0f, 1.0f));  // 2nd rotation
    cvYaw = atan2f(q0 * q3 - q1 * q2, -0.5f + (q1 * q1 + q0 * q0));         // 3rd rotation

    currentYawTest = cvYaw;
    currentPitchTest = current_pitch;
    // express the body-fixed velocities in the correct convention
    bodyXangVel = -drivers->bmi088.getGx() * PI / 180;
    bodyYangVel = -drivers->bmi088.getGy() * PI / 180;
    bodyZangVel = drivers->bmi088.getGz() * PI / 180;

    // convert body-fixed angular velocities into euler angle velocities
    //cvRollVel = sinf(cvYaw) * bodyYangVel + cosf(cvPitch) * cosf(cvYaw) * bodyXangVel;
    //cvPitchVel = cosf(cvYaw) * bodyYangVel - cosf(cvPitch) * sinf(cvYaw) * bodyXangVel;
    cvYawVel = bodyZangVel;

    //get position of camera relative to shooting axis

    posXrel4 = -msg->x + cameraXoffset;
    posYrel4 = msg->z + cameraYoffset;  // taproot flips z y basis vec
    posZrel4 = msg->y + cameraZoffset;
    velXrel4 = -msg->v_x;
    velYrel4 = msg->v_z;
    velZrel4 = msg->v_y;
    //precompute commonly used angles

    float cos_theta3 = cosf(cvYaw);
    float sin_theta3 = sinf(cvYaw);
    float cos_theta4 = cosf(-current_pitch); //reversed due to frame of reference
    float sin_theta4 = sinf(-current_pitch);
    current_pitch_velo *= -1;

    //convert camera offset into reference frame 2
    posXrelPitch =  cos_theta3 * posXrel4 + sin_theta3*(-cos_theta4 * posYrel4 + sin_theta4 * posZrel4);
    posYrelPitch =  sin_theta3 * posXrel4 + cos_theta3*(cos_theta4 * posYrel4 - sin_theta4 * posZrel4);
    posZrelPitch =  sin_theta4 * posYrel4 + cos_theta4 * posZrel4;

    //get the velocity of the camera in reference frame 2
    velXrelPitch =  (-sin_theta3 * posXrel4 - cos_theta3 * (cos_theta4 * posYrel4 - sin_theta4 * posZrel4)) * cvYawVel
    + sin_theta3 * (sin_theta4 * posYrel4 + cos_theta4 * posZrel4) * current_pitch_velo
    + cos_theta3 * velXrel4
    - sin_theta3 * (cos_theta4 * velYrel4 - sin_theta4 * velZrel4);

    velYrelPitch = (cos_theta3 * posXrel4 - sin_theta3 * (cos_theta4 * posYrel4 - sin_theta4 * posZrel4)) * cvYawVel
    - cos_theta3 * (sin_theta4 * posYrel4 + cos_theta4 * posZrel4) * current_pitch_velo
    + sin_theta3 * velXrel4
    + cos_theta3 * (cos_theta4 * velYrel4 - sin_theta4 * velZrel4);

    velZrelPitch = current_pitch_velo * cos_theta4 * posYrel4
    + sin_theta4 * velYrel4
    - current_pitch_velo * sin_theta4 * posZrel4
    + cos_theta4 * velZrel4;

    //get the velocity of the panel in 2

    modm::Vector3f position(posYrelPitch, -posXrelPitch, posZrelPitch);  //X is down range

    //velocity doesn't work well until the latency of the pipeline is significantly reduced    
    modm::Vector3f velocity(velYrelPitch / 4.0f, -velXrelPitch/ 4.0f, velZrelPitch / 100.0f);
    modm::Vector3f acceleration(msg->a_x, msg->a_z, -msg->a_y); //not imlemented yet afak

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

    yawouttest = (targetYaw - cvYaw);
    yawtest2 = targetYaw;


    if (!valid) {
        *action = -1;  // make enums for action
        return;
    }

    *yawOut = (targetYaw - cvYaw);  // fmod(current_yaw + targetYaw, 2 * PI);
    *pitchOut = targetPitch;
    *yawVelOut = (-cos_theta3*velXrelPitch - sin_theta3*velYrelPitch + velXrel4) / (cos_theta4 * posYrel4 - sin_theta4 * posZrel4);

  

    if (abs(*yawOut) < PI / 4) {
        // Enable shooting
        *action = 1;
        return;
    }
    *action = 0;
}

};  // namespace subsystems