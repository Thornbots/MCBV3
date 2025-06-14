#pragma once
#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"
#include "util/Pose2d.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"

#include "drivers.hpp"

using namespace communication;
using namespace tap::algorithms::ballistics;

namespace subsystems {

enum UartMessage : uint8_t{
    // incoming
    ROS_MSG = 0,
    CV_MSG = 1,

    // outgoing
    POSE_MSG = 2,
    REF_SYS_MSG = 3,
};

// =================== Incoming message types =======================

struct ROSData
{
    float x = 0;
    float y = 0;
    float theta = 0;
    float rho = 0;
};

struct CVData 
{
    float x = 0;          // meters
    float y = 0;          // meters
    float z = 0;          // meters
    float v_x = 0;        // m/s
    float v_y = 0;        // m/s
    float v_z = 0;        // m/s
    float a_x = 0;        // m/s^2
    float a_y = 0;        // m/s^2
    float a_z = 0;        // m/s^2
    float confidence = 0; // 0.0 to 1.0
    // uint64_t timestamp = 0;
};

// =================== Output message types =======================

struct PoseData
{
    float x;                     
    float y;                     
    float vel_x;                     
    float vel_y;                     
    float head_pitch;                     
    float head_yaw;                     
    float imu_roll;
    float imu_pitch;
    float imu_yaw;
    float imu_Ax;
    float imu_Ay;
    float imu_Az;
    // uint64_t timestamp = 0;         
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
template<> struct StructToMessageType<ROSData> { static constexpr UartMessage value = ROS_MSG; };
template<> struct StructToMessageType<CVData> { static constexpr UartMessage value = CV_MSG; };
template<> struct StructToMessageType<PoseData> { static constexpr UartMessage value = POSE_MSG; };
template<> struct StructToMessageType<RefSysMsg> { static constexpr UartMessage value = REF_SYS_MSG; };


struct PanelData {
    double r;
    double theta;
};

class JetsonSubsystem : public tap::control::Subsystem {
private:  // Private Variables
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;

    tap::arch::MilliTimeout refDataSendingTimeout;
    // bool needToSendRefData = false;
    static constexpr int TIME_FOR_REF_DATA = 100; //send at 10hz


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
public:  // Public Methods
    JetsonSubsystem(src::Drivers* drivers, GimbalSubsystem* gimbal);

    ~JetsonSubsystem() {}

    void initialize();

    void refresh() override;

    bool updateROS(Vector2d* targetPosition, Vector2d* targetVelocity);
    void update(float current_yaw, float current_pitch, float current_yaw_velo, float current_pitch_velo, float* yawOut, float* pitchOut, float* yawVelOut, float* pitchVelOut, int* action);

    


private:  // Private Methods

    // Constants
    //const float g = 9.81;           // gravitational acceleration
 
        // Height rejection offset
    std::vector<PanelData> panelData;


    template<class msg_type> 
    inline bool getMsg(msg_type* output){
        if(!drivers->uart.hasNewMessage())
            return false;
        const UARTCommunication::cleanedData msg = drivers->uart.getLastMsg();
        if (msg.messageType != StructToMessageType<msg_type>::value || msg.dataLength != sizeof(msg_type))
            return false;
        memcpy(output, (uint8_t*) msg.data, msg.dataLength);
        return true;
    }

    template<class msg_type> 
    inline bool sendMsg(msg_type* msg){
        bool status = drivers->uart.isFinishedWriting();
        if(status)
            return drivers->uart.sendMsg((uint8_t*)msg, StructToMessageType<msg_type>::value, sizeof(msg_type));
        return false;
    }

};
}  // namespace subsystems