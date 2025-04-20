#pragma once
#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"

#include "communication/JetsonCommunication.hpp"

#include "drivers.hpp"

using namespace communication;
using namespace tap::algorithms::ballistics;

namespace subsystems {
struct PanelData {
    double r;
    double theta;
};

class ComputerVisionSubsystem : public tap::control::Subsystem {
private:  // Private Variables
    JetsonCommunication comm;


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
    ComputerVisionSubsystem(tap::Drivers* drivers);

    ~ComputerVisionSubsystem() {}

    void initialize();

    void refresh() override;

    void update(float current_yaw, float current_pitch, float current_yaw_velo, float current_pitch_velo, float* yawOut, float* pitchOut, float* yawVelOut, float* pitchVelOut, int* action);

    


private:  // Private Methods

    // Constants
    //const float g = 9.81;           // gravitational acceleration
 
        // Height rejection offset
    std::vector<PanelData> panelData;
};
}  // namespace subsystems