#include "OdometrySubsystem.hpp"

#include "tap/communication/serial/ref_serial_data.hpp"

#include "OdometrySubsystemConstants.hpp"

namespace subsystems {
    using namespace odo;

using namespace tap::communication::serial;
float encoderOffsetOdo = ODO_OFFSET;
OdometrySubsystem::OdometrySubsystem(src::Drivers* drivers, tap::motor::DjiMotor* odo) : tap::control::Subsystem(drivers), drivers(drivers), motorOdo(odo) {
    distOdo = std::uniform_int_distribution<>(-ODO_DIST_RANGE, ODO_DIST_RANGE);
}

void OdometrySubsystem::initialize() {
    motorOdo->initialize();

    drivers->commandScheduler.registerSubsystem(this);
}
bool useController = false;
void OdometrySubsystem::refresh() {
    bool blue = drivers->refSerial.isBlueTeam(drivers->refSerial.getRobotData().robotId);
    updateAimPosition();
    isBlue = blue;

    if(!useController){
        motorOdo->setDesiredOutput(odoMotorVoltage);
    }
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
int OdometrySubsystem::getOdoVoltage(
    [[maybe_unused]] float driveTrainAngularVelocity,
    [[maybe_unused]] float odoAngleRelativeWorld,
    [[maybe_unused]] float odoAngularVelocity,
    [[maybe_unused]] float desiredAngleWorld,
    [[maybe_unused]] float inputVel,
    [[maybe_unused]] float dt) {
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

float OdometrySubsystem::getOdoEncoderValue() { return std::fmod(motorOdo->getEncoder()->getPosition().getUnwrappedValue() + encoderOffsetOdo, 2 * PI); }

float OdometrySubsystem::getOdoVel() { return motorOdo->getEncoder()->getVelocity(); }

float OdometrySubsystem::getRawX() {
    return drivers->i2c.odom.getX();
}
float OdometrySubsystem::getRawY() {
    return drivers->i2c.odom.getY();
}
float OdometrySubsystem::getStartYaw() {
    return BOOT_YAW;
}
float OdometrySubsystem::getRouteYaw() {
    return isBlue ? PI : 0.0f;
}
Vector2d OdometrySubsystem::fromStart(float forward, float left) {
    Vector2d start(isBlue ? START_X : -START_X, START_Y);
    return start + Vector2d(forward, left).rotate(getRouteYaw());
}
// the pods are x right, y forward of the start: forward is raw y, left is raw -x
void OdometrySubsystem::relocalizeTo(float newX, float newY) {
    if (!std::isfinite(newX) || !std::isfinite(newY)) return;
    updateAimPosition();
    offsetX = newX - aimPosition.getX();
    offsetY = newY - aimPosition.getY();
    navigationLocalized = true;
}
float OdometrySubsystem::getX() {
    return offsetX + getAimX();
}
float OdometrySubsystem::getY() {
    return offsetY + getAimY();
}
// Pods are (right, forward) at boot; rotate (forward, left) into field axes.
void OdometrySubsystem::updateAimPosition() {
    const Vector2d pods(getRawY(), -getRawX());
    aimPosition = aimPosition + (pods - lastPodPosition).rotate(getStartYaw());
    lastPodPosition = pods;
}
float OdometrySubsystem::getAimX() {
    updateAimPosition();
    return aimPosition.getX();
}
float OdometrySubsystem::getAimY() {
    updateAimPosition();
    return aimPosition.getY();
}
float OdometrySubsystem::getXVel() {
    return Vector2d(drivers->i2c.odom.getYVel(), -drivers->i2c.odom.getXVel()).rotate(getStartYaw()).getX();
}
float OdometrySubsystem::getYVel() {
    return Vector2d(drivers->i2c.odom.getYVel(), -drivers->i2c.odom.getXVel()).rotate(getStartYaw()).getY();
}

}  // namespace subsystems
