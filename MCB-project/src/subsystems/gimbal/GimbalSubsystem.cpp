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
    #ifndef OLDINFANTRY
        encoderOffset += drivers->i2c.encoder.getAngle();
    #endif
    imuOffset = getYawEncoderValue();

    targetYawAngleWorld += yawAngleRelativeWorld;
    drivers->commandScheduler.registerSubsystem(this);
}
void GimbalSubsystem::refresh() {
    #ifndef OLDINFANTRY
        if (!motorYaw->isMotorOnline()) {
            encoderOffset = drivers->i2c.encoder.getAngle() + YAW_OFFSET;
            motorYaw->resetEncoderValue();
        }

    #endif

    yawAngularVelocity = PI / 180 * drivers->bmi088.getGz();

    #if defined(INFANTRY)
        gimbalPitchAngularVelocity = drivers->bmi088.getGx() * PI / 180;
        gimbalPitchAngleRelativeWorld =  drivers->bmi088.getRoll() * PI / 180;
    #endif


    driveTrainAngularVelocity = yawAngularVelocity - getYawVel();
    yawAngleRelativeWorld = PI / 180 * drivers->bmi088.getYaw() - imuOffset;
    motorPitch->setDesiredOutput(pitchMotorVoltage);
    motorYaw->setDesiredOutput(yawMotorVoltage);
}

void GimbalSubsystem::updateMotors(float changeInTargetYaw, float targetPitch) {
    
    float pitchVel = getPitchVel();
    float pitch = getPitchEncoderValue();
    #if defined(INFANTRY)
        targetPitch -= gimbalPitchAngleRelativeWorld;
        pitchVel += gimbalPitchAngularVelocity;
    #endif

    targetPitch = std::clamp(targetPitch , -MAX_PITCH_DOWN, MAX_PITCH_UP);

    driveTrainEncoder = getYawEncoderValue();
    yawEncoderCache = driveTrainEncoder;
    //THIS LINE BELOW WAS CAUSING ERROR
    targetYawAngleWorld += changeInTargetYaw;// std::fmod(targetYawAngleWorld + changeInTargetYaw, 2 * PI);
    pitchMotorVoltage = getPitchVoltage(targetPitch, pitch, pitchVel, dt);

    yawMotorVoltage = getYawVoltage(driveTrainAngularVelocity, yawAngleRelativeWorld, yawAngularVelocity, targetYawAngleWorld, changeInTargetYaw / dt, dt);
    // moved
}

void GimbalSubsystem::stopMotors() {
    pitchMotorVoltage = 0;
    yawMotorVoltage = 0;

    pitchController.clearBuildup();
    yawController.clearBuildup();
}

void GimbalSubsystem::reZeroYaw() {
    // TODO
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
    return 1000 * pitchController.calculate(pitchAngleRelativeGimbal, pitchAngularVelocity, targetAngle + PITCH_OFFSET, dt);
#endif
}

float GimbalSubsystem::getYawEncoderValue() { return std::fmod(motorYaw->getPositionUnwrapped() / YAW_TOTAL_RATIO + encoderOffset, 2 * PI); }

float GimbalSubsystem::getPitchEncoderValue() { return motorPitch->getPositionWrapped(); }
float GimbalSubsystem::getYawVel() { return motorYaw->getShaftRPM() * PI / 30 / YAW_TOTAL_RATIO; }
float GimbalSubsystem::getPitchVel() { return motorPitch->getShaftRPM() * PI / 30; }

}  // namespace subsystems