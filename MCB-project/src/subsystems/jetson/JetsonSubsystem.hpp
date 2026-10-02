#pragma once
#include <array>

#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"
#include "util/Pose2d.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/odometry/OdometrySubsystem.hpp"
#include "subsystems/ui/objects/HitRing.hpp"

#include "drivers.hpp"

using namespace communication;
using namespace tap::algorithms::ballistics;

namespace subsystems {

enum UartMessage : uint8_t{
    // incoming
    NAV_GOAL_MSG = 0,
    CV_TARGET_MSG = 1,
    RELOCALIZE = 4,

    // outgoing
    POSE_MSG = 2,
    REF_SYS_MSG = 3,

};


enum OdomStatus : uint8_t{
    ODOM_PODS = 0,                  // odometry pods healthy, best accuracy
    ODOM_DRIVETRAIN = 1,            // drivetrain odometry, degraded by wheel slip
    ODOM_I2C_DEAD = 2,              // I2C bus dead, no pod data, no fallback
    ODOM_I2C_DEAD_DRIVETRAIN = 3    // I2C bus dead, drivetrain odometry instead
};

// =================== Incoming message types =======================

// where sentry wants to go
struct NavGoal
{
    float targetX = 0; //meters
    float targetY = 0; //meters
};

struct CVTarget
{
    float x = 0;           // meters
    float y = 0;           // meters
    float z = 0;           // meters
    uint8_t booleans = 0;
    // bool shoot;            //value 1: 0 is no shoot, 1 is shoot
    // bool typeCBasedPatrol; //value 2: 0 disables patrolling, 1 allows patrolling
    // bool turnToHit;        //value 4: 0 disables turning to the direction we got hit in, 1 allows it
    // bool unused;           //value 8:
    // bool unused;           //value 16:
    // bool unused;           //value 32:
    // bool unused;           //value 64:
    // bool unused;           //value 128:
    uint16_t delay_ms = 0; //from when we receive this to this time, a shot needs to be fired. Might not be used?
};

struct Relocalize
{
    //where I think I am, by the lidar
    float expectedX = 0; //meters
    float expectedY = 0; //meters
};

// =================== Output message types =======================

struct PoseData
{
    float x;          //meters
    float y;          //meters
    float vel_x;      //meters/second
    float vel_y;      //meters/second
    float head_pitch; //rad
    float head_yaw;   //rad
    OdomStatus error_code;
} modm_packed;
// static_assert(sizeof(PoseData)<1024, "msg too large"); //TODO: implement static check

struct RefSysMsg
{
    uint8_t gameStage;
    uint16_t stageTimeRemaining;
    uint16_t robotHp;
    uint8_t robotID; //if was on red team, so hero will always be 1 and not 101
    float deltaAngleGotHitIn; //if we are looking in a certain direction and get hit in the left, this would be PI/2

    uint8_t booleans;
    // bool isOnBlueTeam;
    // bool isHealing;
    // bool isInReloadZone;
    // bool isInCenterZone;
    // bool doesTeamOccupyCenterZone;
    // bool doesOpponentTeamOccupyCenterZone;
    // bool doesChassisHavePower;
    // bool doesGimbalHavePower;
} modm_packed;

// ==== struct type to enum mapping ===
template<typename T>
struct StructToMessageType;
template<> struct StructToMessageType<NavGoal> { static constexpr UartMessage value = NAV_GOAL_MSG; };
template<> struct StructToMessageType<CVTarget> { static constexpr UartMessage value = CV_TARGET_MSG; };
template<> struct StructToMessageType<PoseData> { static constexpr UartMessage value = POSE_MSG; };
template<> struct StructToMessageType<RefSysMsg> { static constexpr UartMessage value = REF_SYS_MSG; };
template<> struct StructToMessageType<Relocalize> { static constexpr UartMessage value = RELOCALIZE; };

// Snapshot of the turret orientation (IMU-derived world-frame yaw/pitch and their rates) taken
// once per control cycle. These are queued in a fixed-length delay line so that a CV frame, which
// arrives with pipeline latency, can be transformed into the world frame using the orientation as
// it was when that frame was actually captured rather than the live (newer) orientation.
struct OrientationSample {
    float cvYaw = 0;       // world-frame turret yaw   (XYZ-euler 3rd rotation), rad
    float cvPitch = 0;     // world-frame turret pitch (XYZ-euler 2nd rotation), rad
    float cvYawVel = 0;    // yaw rate, rad/s
    float cvPitchVel = 0;  // pitch rate, rad/s
};

class JetsonSubsystem : public tap::control::Subsystem {
private:  // Private Variables
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;
    OdometrySubsystem* odo;
    HitRing hitRing{drivers, gimbal};

    static constexpr int TIME_FOR_REF_DATA = 200; //send at 5hz
    tap::arch::PeriodicMilliTimer refDataSendingTimeout{TIME_FOR_REF_DATA};
    tap::arch::PeriodicMilliTimer poseDataTimeout{10}; //for sending pose data at 100hz
    // bool needToSendRefData = false;


    float q0, q1, q2, q3; //easier to convert frames of reference from the quatrenion directly
    float cvRoll, cvPitch, cvYaw; //expressed in XYZ euler angles, not the IMU's standard ZYX
    float cvRollVel, cvPitchVel, cvYawVel;
    float bodyXangVel, bodyYangVel, bodyZangVel;
    float imuGx;
    float imuGy;
    float imuGz;
   
    float posXrel4, posYrel4, posZrel4; //position of the panel relative to the 4th frame aka the shooter axis
    float velXrel4, velYrel4, velZrel4;
    float posXrelPitch, posYrelPitch, posZrelPitch; //position of panel relative to frame 2 but offset up
    float velXrelPitch, velYrelPitch, velZrelPitch;

    // ---- orientation delay line for CV latency compensation ----
    // Number of control cycles of orientation history to buffer. The transform reads the sample
    // from the tail of the queue (the oldest one held), so this length sets the compensated
    // latency: delay ~= ORIENTATION_QUEUE_SIZE * controlCyclePeriod. Tune so that delay matches
    // the combined camera + Jetson + transport latency of a CV frame.
    static constexpr size_t ORIENTATION_QUEUE_SIZE = 23; // 1 isaffects 'resonating', where if it starts pointed at it, it gets worse and bounces side ot side
    std::array<OrientationSample, ORIENTATION_QUEUE_SIZE> orientationQueue{};
    size_t orientationQueueHead = 0;  // index of the oldest sample == next slot to overwrite

public:  // Public Methods
    JetsonSubsystem(src::Drivers* drivers, GimbalSubsystem* gimbal, OdometrySubsystem* odo);

    ~JetsonSubsystem() {}

    void initialize();

    void refresh() override;
    
    // check if jetson sent a relocalize message. If it did, overwrite where odo thinks it is
    void checkApplyRelocalize();

    bool updateROS(Vector2d* targetPosition, Vector2d* targetVelocity, Vector2d* jetsonExpectedPosition);
    void update(float current_yaw, float current_pitch, float current_yaw_velo, float current_pitch_velo, float* yawOut, float* pitchOut, float* yawVelOut, float* pitchVelOut, int* action);

    // AutoAimAndFireCommand knows how to interpret the CVTarget message
    bool getCVTarget(CVTarget* cvTarget);
    
    float getAngleToTurnForSentry();


private:  // Private Methods

    // Sample the current turret orientation from the IMU and push it into the delay line.
    // Call exactly once per control cycle (from refresh()).
    void recordOrientationSample();

    // Returns the orientation held at the tail of the delay line, i.e. the turret orientation
    // from ~ORIENTATION_QUEUE_SIZE cycles ago, used to compensate for CV pipeline latency.
    const OrientationSample& getDelayedOrientation() const;

    // Fill the data of the message with the most recently received message. Returns true if the message was updated, false if not.
    template<class msg_type>
    inline bool getMsg(msg_type* output){
        if(!drivers->uart.hasNewMessage())
            return false;
        const UARTCommunication::uartMsg msg = drivers->uart.getLastMsg();
        if (msg.messageType != StructToMessageType<msg_type>::value || msg.dataLength != sizeof(msg_type))
            return false;
        memcpy(output, (uint8_t*) msg.data, msg.dataLength);
        drivers->uart.clearNewDataFlag();
        return true;
    }

    template<class msg_type> 
    inline bool sendMsg(msg_type* msg){
        bool status = drivers->uart.isFinishedWriting(); //TODO: this is not necessary because uart has a buffer?
        if(status)
            return drivers->uart.sendMsg((uint8_t*)msg, StructToMessageType<msg_type>::value, sizeof(msg_type));
        return false;
    }
    
    
    float angleToTurnForSentry;

};
}  // namespace subsystems