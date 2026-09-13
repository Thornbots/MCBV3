#include "OdometrySubsystem.hpp"

#include "tap/communication/serial/ref_serial_data.hpp"

#include "OdometrySubsystemConstants.hpp"

namespace subsystems {
    using namespace odo;

using namespace tap::communication::serial;
float encoderOffsetOdo = ODO_OFFSET;
OdometrySubsystem::OdometrySubsystem(src::Drivers* drivers, tap::motor::DjiMotor* odo) : tap::control::Subsystem(drivers), drivers(drivers), motorOdo(odo) {
    gen = std::mt19937(rd());
    distOdo = std::uniform_int_distribution<>(-ODO_DIST_RANGE, ODO_DIST_RANGE);
}

void OdometrySubsystem::initialize() {
    motorOdo->initialize();

    // targetOdoAngleWorld = odoAngleRelativeWorld;
    drivers->commandScheduler.registerSubsystem(this);
}
bool useController = false;
void OdometrySubsystem::refresh() {

    if(!useController){
        motorOdo->setDesiredOutput(odoMotorVoltage);
    }
    // odoAngleRelativeWorld = PI / 180 * drivers->bmi088.getYaw() - getOdoEncoderValue();
}

void OdometrySubsystem::updateMotor(float targetOdo, float odoAngleRelativeWorld, float odoVelRelativeWorld, float driveTrainAngularVelocity) {
    useController = true;
    odoMotorVoltage = getOdoVoltage(driveTrainAngularVelocity, std::fmod(odoAngleRelativeWorld, 2 * PI), odoVelRelativeWorld, targetOdo, 0, dt);
    motorOdo->setDesiredOutput(odoMotorVoltage);

}

void OdometrySubsystem::stopMotors() {
    useController = false;
    odoMotorVoltage = 0;

    odoController.clearBuildup();
}


// assume odoAngleRelativeWorld is in radians, not sure
int OdometrySubsystem::getOdoVoltage(float driveTrainAngularVelocity, float odoAngleRelativeWorld, float odoAngularVelocity, float desiredAngleWorld, float inputVel, float dt) {
#if defined(odo_sysid)
    voltageOdo = distOdo(gen);
    velocityOdo = odoAngularVelocity;
    return voltageOdo;
#elif defined(drivetrain_sysid)
    return 0;
#else
    return 1000 * odoController.calculate(odoAngleRelativeWorld, odoAngularVelocity, driveTrainAngularVelocity, desiredAngleWorld, inputVel, dt);
#endif
}

float OdometrySubsystem::getOdoEncoderValue() { return std::fmod(motorOdo->getPositionUnwrapped() + encoderOffsetOdo, 2 * PI); }

float OdometrySubsystem::getOdoVel() { return motorOdo->getShaftRPM() * PI / 30; }

float OdometrySubsystem::getRawX() {
    return drivers->i2c.odom.getX();
}
float OdometrySubsystem::getRawY() {
    return drivers->i2c.odom.getY();
}
void OdometrySubsystem::relocalizeTo(float newX, float newY) {
    offsetX = newX - getRawX();
    offsetY = newY - getRawY();
    //off       = new - raw
    //off + raw = new
}
float OdometrySubsystem::getX() {
    return offsetX + getRawX();
}
float OdometrySubsystem::getY() {
    return offsetY + getRawY();
}
float OdometrySubsystem::getXVel() {
    return drivers->i2c.odom.getXVel();
}
float OdometrySubsystem::getYVel() {
    return drivers->i2c.odom.getYVel();
}

}  // namespace subsystems