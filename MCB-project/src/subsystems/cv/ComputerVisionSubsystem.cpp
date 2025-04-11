#include "ComputerVisionSubsystem.hpp"
// #include "ComputerVisionSubsystemConstants.hpp"

namespace subsystems {

ComputerVisionSubsystem::ComputerVisionSubsystem(tap::Drivers* drivers) :
     tap::control::Subsystem(drivers), comm(drivers, tap::communication::serial::Uart::Uart1, true) {}

void ComputerVisionSubsystem::initialize() {
    drivers->commandScheduler.registerSubsystem(this);
    comm.initialize();
}

void ComputerVisionSubsystem::refresh() { 
    comm.updateSerial(); 
}

void ComputerVisionSubsystem::update(float current_yaw, float current_pitch, float* yawOut, float* pitchOut, int* action) {
    const CVData* msg = comm.getLastCVData();
    if(msg == nullptr){
        *action = -1;
        return;
    }
    
    comm.clearNewDataFlag();
     // Add rotated offset vector of panel relative to RGB
    // if (msg->confidence <= 0.2f) return;

    // float X_prime = -x + 0.0175;                                                     // left
    // float Y_prime = -y + 0.1295 * cos(current_pitch) - 0.0867 * sin(current_pitch);  // up
    // float Z_prime = z + 0.0867 * cos(current_pitch) + 0.1295 * sin(current_pitch);   // forwards


    //TODO: just store vec3s in JetosnComms msg struct
    // modm::Vector3f pos(msg->x,msg->y,msg->z);
    // modm::Vector3f vel(msg->v_x,msg->v_y,msg->v_z);
    // modm::Vector3f acc(msg->a_x,msg->a_y,msg->a_z);


    // MeasuredKinematicState state;//(pos,vel,acc); 
    // state.position = modm::Vector3f(0,0,0);
    // state.velocity = modm::Vector3f(0,0,0);
    // state.acceleration = modm::Vector3f(0,0,0);
    modm::Vector3f position(msg->x,msg->z + 0.2,-msg->y - 0.7); // taproot flips z y basis vec
    modm::Vector3f velocity(msg->v_x,msg->v_z,-msg->v_y);
    modm::Vector3f acceleration(msg->a_x,msg->a_z,-msg->a_y);


    SecondOrderKinematicState state(position, velocity, acceleration);//(pos,vel,acc); 

    float targetYaw, targetPitch, travelTime;
    bool valid = tap::algorithms::ballistics::findTargetProjectileIntersection(state, J, 2, &targetPitch, &targetYaw, &travelTime, 0);

    if(!valid){
        *action = -1;// make enums for action
        return;
    }

    *yawOut = targetYaw-PI/2;// fmod(current_yaw + targetYaw, 2 * PI);
    *pitchOut = targetPitch;

    if (abs(*yawOut) < PI/16 ) {
        // Enable shooting
        *action = 1;
        return;
    }
    *action = 0;
}   
};
  // namespace subsystems