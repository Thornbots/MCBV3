#include "GimbalSubsystem.hpp"

#include "tap/communication/serial/ref_serial_data.hpp"

#include "GimbalSubsystemConstants.hpp"

int voltage;
float velocity;
namespace subsystems {
using namespace tap::communication::serial;
float encoderOffset = YAW_OFFSET;
GimbalSubsystem::GimbalSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* yaw, tap::motor::DjiMotor* pitch) : tap::control::Subsystem(drivers), drivers(drivers), motorYaw(yaw), motorPitch(pitch) {
    gen = std::mt19937(rd());
    distYaw = std::uniform_int_distribution<>(-YAW_DIST_RANGE, YAW_DIST_RANGE);
    distPitch = std::uniform_int_distribution<>(-PITCH_DIST_RANGE, PITCH_DIST_RANGE);
}

void GimbalSubsystem::initialize() {
    motorPitch->initialize();
    motorYaw->initialize();
    imuOffset = getYawEncoderValue();

    targetYawAngleWorld += yawAngleRelativeWorld;
    drivers->commandScheduler.registerSubsystem(this);
}
void GimbalSubsystem::refresh() {

    yawAngularVelocity = PI / 180 * drivers->bmi088.getGz();

#if defined(INFANTRY) or defined(HERO)
    gimbalPitchAngularVelocity = drivers->bmi088.getGx() * PI / 180;

    // this happens to work because the X axis is alined with the pitch axis
    gimbalPitchAngleRelativeWorld = drivers->bmi088.getRoll() * PI / 180;

#endif

    driveTrainAngularVelocity = yawAngularVelocity - getYawVel();
    yawAngleRelativeWorld = PI / 180 * drivers->bmi088.getYaw(); 
    updatePositionHistory(yawAngleRelativeWorld);
    motorPitch->setDesiredOutput(pitchMotorVoltage);
    motorYaw->setDesiredOutput(yawMotorVoltage);
}

void GimbalSubsystem::updateMotors(float changeInTargetYaw, float targetPitch) {
    float pitchVel = getPitchVel();
    float pitch = getPitchEncoderValue();
    prevTargetPitch = std::clamp(targetPitch, -MAX_PITCH_DOWN, MAX_PITCH_UP);
    
#if defined(INFANTRY) or defined(HERO) // chicken mode, gets bad when imu drifts
    targetPitch -= gimbalPitchAngleRelativeWorld;
    pitchVel += gimbalPitchAngularVelocity;
#endif

    targetPitch = std::clamp(targetPitch, -MAX_PITCH_DOWN, MAX_PITCH_UP);

    driveTrainEncoder = getYawEncoderValue();
    yawEncoderCache = driveTrainEncoder;
    // THIS LINE BELOW WAS CAUSING ERROR
    targetYawAngleWorld += changeInTargetYaw;  // std::fmod(targetYawAngleWorld + changeInTargetYaw, 2 * PI);
    pitchMotorVoltage = getPitchVoltage(targetPitch, pitch, pitchVel, dt);

    yawMotorVoltage = getYawVoltage(driveTrainAngularVelocity, yawAngleRelativeWorld, yawAngularVelocity, targetYawAngleWorld, changeInTargetYaw / dt, dt);
    // moved
}

float GimbalSubsystem::getPrevTargetPitch() {
    return prevTargetPitch;
}

// alternate version of update motors to use with CV
void GimbalSubsystem::updateMotorsAndVelocity(float changeInTargetYaw, float targetPitch, float targetYawVel, float targetPitchVel) {
    float pitch = getPitchEncoderValue();

    prevTargetPitch = std::clamp(targetPitch, -MAX_PITCH_DOWN, MAX_PITCH_UP);

    driveTrainEncoder = getYawEncoderValue();
    yawEncoderCache = driveTrainEncoder;
    // THIS LINE BELOW WAS CAUSING ERROR
    targetYawAngleWorld += changeInTargetYaw;  // std::fmod(targetYawAngleWorld + changeInTargetYaw, 2 * PI);
    pitchMotorVoltage = getPitchVoltage(prevTargetPitch, pitch, targetPitchVel, dt);

    yawMotorVoltage = getYawVoltage(driveTrainAngularVelocity, yawAngleRelativeWorld, yawAngularVelocity, targetYawAngleWorld, targetYawVel, dt);
}

//latency compensation to improve yaw tracking
void GimbalSubsystem::updateMotorsAndVelocityWithLatencyCompensation(float changeInTargetYaw, float targetPitch, float targetYawVel, float targetPitchVel){
    float newChangeInTargetYaw = changeInTargetYaw - yawAngleRelativeWorld + positionHistory[LATENCY_Q_SIZE - 1]; //offsets change in yaw based on how far the yaw has moved since the camera captures a frame
    updateMotorsAndVelocity(newChangeInTargetYaw, targetPitch, targetYawVel, targetPitchVel);

}

void GimbalSubsystem::stopMotors() {
    pitchMotorVoltage = 0;
    yawMotorVoltage = 0;
    
    // #if defined(INFANTRY) or defined(HERO) or defined(SENTRY)  //all robots with 3508 turrets
    if (!motorYaw->isMotorOnline() || !drivers->remote.isConnected()) {
        encoderOffset = drivers->i2c.encoder.getAngle() + YAW_OFFSET;
        motorYaw->resetEncoderValue();
    }
    // #endif
    targetYawAngleWorld = yawAngleRelativeWorld;

    clearBuildup();
}

void GimbalSubsystem::clearBuildup() {
    pitchController.clearBuildup();
    yawController.clearBuildup();
}
    
void GimbalSubsystem::reZeroYaw() {
    // TODO
}

void GimbalSubsystem::updatePositionHistory(float newPos) {
    for (int i = LATENCY_Q_SIZE - 1; i >= 0; i--) {
        // Store the current values in the history
        // load in new value if i is not > 0
        // flast element in the queue gets kicked out every call
        positionHistory[i] = (i > 0) ? positionHistory[i - 1] : newPos;

    }
}

// assume yawAngleRelativeWorld is in radians, not sure
int GimbalSubsystem::getYawVoltage(float driveTrainAngularVelocity, float yawAngleRelativeWorld, float yawAngularVelocity, float desiredAngleWorld, float inputVel, float dt) {
#if defined(yaw_sysid)
    voltage = distYaw(gen);
    velocity = yawAngularVelocity;
    return voltage;
#elif defined(drivetrain_sysid)
    return 0;
#else
    return 1000 * yawController.calculate(yawAngleRelativeWorld, yawAngularVelocity, driveTrainAngularVelocity, desiredAngleWorld, inputVel, dt);
#endif
}

// assume targetangle is in radians, not sure
int GimbalSubsystem::getPitchVoltage(float targetAngle, float pitchAngleRelativeGimbal, float pitchAngularVelocity, float dt) {
#if defined(pitch_sysid)
    return distPitch(gen);
#elif defined(drivetrain_sysid)
    return 0;
#else
    return 1000 * pitchController.calculate(pitchAngleRelativeGimbal, pitchAngularVelocity, targetAngle, dt);
#endif
}

float GimbalSubsystem::getYawEncoderValue() { return std::fmod(motorYaw->getPositionUnwrapped() / YAW_TOTAL_RATIO + encoderOffset, 2 * PI); }

float GimbalSubsystem::getPitchEncoderValue() { //more like get pitch relative to frame
    float temp = std::fmod(motorPitch->getPositionWrapped() / PITCH_RATIO - PITCH_OFFSET, 2 * PI);
    #if defined(HERO) //wraparound fix  
    return (temp > (1.3 * PI/PITCH_RATIO)) ? temp - 2 * PI/PITCH_RATIO : temp;
    #else
    return (temp > PI) ? temp - 2 * PI : temp;
    #endif
}
float GimbalSubsystem::getYawVel() { return motorYaw->getShaftRPM() * PI / 30 / YAW_TOTAL_RATIO; }
float GimbalSubsystem::getPitchVel() { return motorPitch->getShaftRPM() * PI / 30; }
float GimbalSubsystem::getYawAngleRelativeWorld() { return yawController.estimatedPosition; }
}  // namespace subsystems  