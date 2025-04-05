#include "OdometrySubsystem.hpp"

#include "OdometrySubsystemConstants.hpp"

// int voltage;
// float velocity;
namespace subsystems {
float odoEncoderOffset = 0;
OdometrySubsystem::OdometrySubsystem(src::Drivers* drivers, tap::motor::DjiMotor* motorOdometry) : tap::control::Subsystem(drivers), drivers(drivers), motorOdometry(motorOdometry) {
    gen = std::mt19937(rd());
    distOdometry = std::uniform_int_distribution<>(-ODOMETRY_DIST_RANGE, ODOMETRY_DIST_RANGE);
}

void OdometrySubsystem::initialize() {
    motorOdometry->initialize();
#ifndef OLDINFANTRY
    odoEncoderOffset = drivers->i2c.encoder.getAngle();
#endif
    imuOffset = getOdometryEncoderValue() + ODOMETRY_OFFSET;

    targetOdometryAngleWorld += odometryAngleRelativeWorld;
    drivers->commandScheduler.registerSubsystem(this);
}

void OdometrySubsystem::refresh() {
#ifndef OLDINFANTRY
    if (!motorOdometry->isMotorOnline()) {
        odoEncoderOffset = drivers->i2c.encoder.getAngle();
        // motorOdometry->resetEncoderValue();
    }

#endif
    odometryAngularVelocity = PI / 180 * drivers->bmi088.getGz();
    driveTrainAngularVelocity = odometryAngularVelocity - getOdometryVel();
    odometryAngleRelativeWorld = fmod(PI / 180 * drivers->bmi088.getYaw() - imuOffset, 2 * PI);
}

void OdometrySubsystem::updateMotor(float changeInTargetOdometry) {
    driveTrainEncoder = getOdometryEncoderValue();
    odometryEncoderCache = driveTrainEncoder;
    targetOdometryAngleWorld = fmod(targetOdometryAngleWorld + changeInTargetOdometry, 2 * PI);
    odometryMotorVoltage = getOdometryVoltage(driveTrainAngularVelocity, odometryAngleRelativeWorld, odometryAngularVelocity, targetOdometryAngleWorld, changeInTargetOdometry / dt, dt);
    motorOdometry->setDesiredOutput(odometryMotorVoltage);
}

void OdometrySubsystem::stopMotors() {
    odometryMotorVoltage = 0;
    motorOdometry->setDesiredOutput(odometryMotorVoltage);
    odometryController.clearBuildup();
}

int OdometrySubsystem::getOdometryVoltage(float driveTrainAngularVelocity, float odometryAngleRelativeWorld, float odometryAngularVelocity, float desiredAngleWorld, float inputVel, float dt) {
#if defined(odo_sysid)
    voltage = distOdometry(gen);
    velocity = odometryAngularVelocity;
    return voltage;
#elif defined(drivetrain_sysid)
    return 0;
#else
    return 1000 * odometryController.calculate(odometryAngleRelativeWorld, odometryAngularVelocity, driveTrainAngularVelocity, desiredAngleWorld, inputVel, dt);
#endif
}

float OdometrySubsystem::getOdometryEncoderValue() { return motorOdometry->getPositionUnwrapped() / ODOMETRY_TOTAL_RATIO + odoEncoderOffset; }

float OdometrySubsystem::getOdometryVel() { return motorOdometry->getShaftRPM() * PI / 30 / ODOMETRY_TOTAL_RATIO; }

};  // namespace subsystems
    // namespace subsystems
