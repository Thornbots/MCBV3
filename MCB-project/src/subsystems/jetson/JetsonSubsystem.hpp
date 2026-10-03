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

// Named after the Jetson's ROS topic each one carries. Wire layouts:
// ros2_dji_serial_bridge's UART_PROTOCOL.md, the Jetson's side of this file.
enum UartMessage : uint8_t{
    // incoming
    NAV_GOAL = 0,
    CV_TARGET = 1,
    RELOCALIZE = 4,

    // outgoing
    POSE = 2,
    REF_SYS = 3,
    
    // bidirectional
    PING = 5,

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
    float x = 0; //meters
    float y = 0; //meters
};


static constexpr uint8_t CV_TARGET_FLAG_FIRE = 0x01;                // fire delay_ms after receipt
static constexpr uint8_t CV_TARGET_FLAG_TYPE_C_BASED_PATROL = 0x02; // 0 stops patrolling, 1 allows it
static constexpr uint8_t CV_TARGET_FLAG_TURN_TO_HIT = 0x04;         // 0 stops turning toward a hit, 1 allows it
// Aim point and fire decision in one frame. x/y/z is a world-frame point in the
// Jetson's odom (REP-105, z up), not a camera-frame one. delay_ms runs from receipt.
static constexpr uint8_t CV_TARGET_FLAGS_DEFAULT = CV_TARGET_FLAG_TURN_TO_HIT | CV_TARGET_FLAG_TYPE_C_BASED_PATROL;
struct CvTarget
{
    float x = 0;           // meters
    float y = 0;           // meters
    float z = 0;           // meters, up
    uint16_t delay_ms = 0; // fire this many ms after the frame arrives (0 = now)
    uint8_t flags = CV_TARGET_FLAGS_DEFAULT;     // CV_TARGET_FLAG_* bits, bits 3-7 reserved (0)
} modm_packed;

//where lidar thinks the robot is
struct Relocalize
{
    float x = 0; 
    float y = 0;
} modm_packed;


// =================== Output message types =======================

struct Pose
{
    float x;          //meters
    float y;          //meters
    float vel_x;      //meters/second
    float vel_y;      //meters/second
    float head_pitch; //rad
    float head_yaw;   //rad
    OdomStatus odom_status;
} modm_packed;

struct RefSys
{
    uint8_t game_stage;
    uint16_t stage_time_remaining;
    uint16_t robot_hp;
    uint8_t robot_id; //if was on red team, so hero will always be 1 and not 101
    float delta_angle_got_hit_in; //if we are looking in a certain direction and get hit in the left, this would be PI/2

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

struct Ping
{
    uint8_t number=0;
} modm_packed;

// ==== struct type to enum mapping ===
template<typename T>
struct StructToMessageType;
template<> struct StructToMessageType<NavGoal> { static constexpr UartMessage value = NAV_GOAL; };
template<> struct StructToMessageType<CvTarget> { static constexpr UartMessage value = CV_TARGET; };
template<> struct StructToMessageType<Pose> { static constexpr UartMessage value = POSE; };
template<> struct StructToMessageType<RefSys> { static constexpr UartMessage value = REF_SYS; };
template<> struct StructToMessageType<Relocalize> { static constexpr UartMessage value = RELOCALIZE; };
template<> struct StructToMessageType<Ping> { static constexpr UartMessage value = PING; };


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


public:  // Public Methods
    JetsonSubsystem(src::Drivers* drivers, GimbalSubsystem* gimbal, OdometrySubsystem* odo);

    ~JetsonSubsystem() {}

    void initialize();

    void refresh() override;
    
    // check if jetson sent a relocalize message. If it did, overwrite where odo thinks it is
    void checkApplyRelocalize();

    bool updateROS(Vector2d* targetPosition, Vector2d* targetVelocity, Vector2d* jetsonExpectedPosition);

    // AutoAimAndFireCommand knows how to interpret the CvTarget message
    bool getCvTarget(CvTarget* cvTarget);
    
    float getAngleToTurnForSentry();


private:  // Private Methods

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