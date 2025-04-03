#include "GimbalSubsystem.hpp"
#include "GimbalSubsystemConstants.hpp"

int voltage;
float velocity;
namespace subsystems
{

GimbalSubsystem::GimbalSubsystem(tap::Drivers* drivers, tap::motor::DjiMotor* yaw, tap::motor::DjiMotor* pitch) : tap::control::Subsystem(drivers), drivers(drivers), motorYaw(yaw), motorPitch(pitch)
{
    gen = std::mt19937(rd());
    distYaw = std::uniform_int_distribution<>(-YAW_DIST_RANGE, YAW_DIST_RANGE);
    distPitch = std::uniform_int_distribution<>(-PITCH_DIST_RANGE, PITCH_DIST_RANGE);
}

void GimbalSubsystem::initialize()
{
    motorPitch->initialize();
    motorYaw->initialize();
    imuOffset = getYawEncoderValue() + YAW_OFFSET;
    targetYawAngleWorld += yawAngleRelativeWorld;
    drivers->commandScheduler.registerSubsystem(this);
}

void GimbalSubsystem::refresh()
{
    yawAngularVelocity = PI / 180 * drivers->bmi088.getGz();
    driveTrainAngularVelocity = yawAngularVelocity - getYawVel();
    yawAngleRelativeWorld = fmod(PI / 180 * drivers->bmi088.getYaw() - imuOffset, 2 * PI);

}

void GimbalSubsystem::updateMotors(float* changeInTargetYaw, float* targetPitch)
{

    *targetPitch = std::clamp(*targetPitch, -MAX_PITCH_DOWN, MAX_PITCH_UP);
    driveTrainEncoder = getYawEncoderValue();
    yawEncoderCache = driveTrainEncoder;
    targetYawAngleWorld = fmod(targetYawAngleWorld + *changeInTargetYaw, 2 * PI);
    pitchMotorVoltage = getPitchVoltage(*targetPitch, dt);
    yawMotorVoltage = getYawVoltage(driveTrainAngularVelocity, yawAngleRelativeWorld, yawAngularVelocity, targetYawAngleWorld, *changeInTargetYaw / dt, dt);

    //moved     
    motorPitch->setDesiredOutput(pitchMotorVoltage);
    motorYaw->setDesiredOutput(yawMotorVoltage);
}

void GimbalSubsystem::stopMotors()
{
    pitchMotorVoltage = 0;
    yawMotorVoltage = 0;
    motorPitch->setDesiredOutput(pitchMotorVoltage);
    motorYaw->setDesiredOutput(yawMotorVoltage);
    pitchController.clearBuildup();
    yawController.clearBuildup();
}

void GimbalSubsystem::reZeroYaw()
{
    // TODO
}

// assume yawAngleRelativeWorld is in radians, not sure
int GimbalSubsystem::getYawVoltage(float driveTrainAngularVelocity, float yawAngleRelativeWorld, float yawAngularVelocity, float desiredAngleWorld, float inputVel, float dt)
{
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
int GimbalSubsystem::getPitchVoltage(float targetAngle, float dt)
{
#if defined(pitch_sysid)
    return distPitch(gen);
#elif defined(drivetrain_sysid)
    return 0;
#else
    return 1000 * pitchController.calculate(getPitchEncoderValue(), getPitchVel(), targetAngle + PITCH_OFFSET, dt);
#endif
}

float GimbalSubsystem::getYawEncoderValue() { return motorYaw->getEncoder()->getPosition().getWrappedValue() / YAW_TOTAL_RATIO; }
float GimbalSubsystem::getPitchEncoderValue() { return motorPitch->getEncoder()->getPosition().getWrappedValue();}
float GimbalSubsystem::getYawVel() { return motorYaw->getShaftRPM() * PI / 30 / YAW_TOTAL_RATIO; }
float GimbalSubsystem::getPitchVel() { return motorPitch->getShaftRPM() * PI / 30; }

}  // namespace subsystems