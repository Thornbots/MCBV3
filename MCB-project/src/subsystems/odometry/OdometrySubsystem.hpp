#pragma once
#include <random>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

#include "controllers/OdoController.hpp"
#include "drivers.hpp"
#include "util/Vector2d.hpp"




namespace subsystems
{

class OdometrySubsystem : public tap::control::Subsystem
{

private:  // Private Variables
    src::Drivers* drivers;
    tap::motor::DjiMotor* motorOdo;

    OdoController odoController;  // default constructor

    float odoMotorVoltage, driveTrainEncoder, odoEncoderCache, driveTrainAngularVelocity, odoAngleRelativeWorld, odoAngularVelocity;

    static constexpr float targetOdoAngleWorld = 0;
    
    // changes as a result of relocalizeTo, used in getX and getY. Field frame
    float offsetX = 0.0f;
    float offsetY = 0.0f;

    // team from the referee, which picks the start; red until it says blue
    bool isBlue = false;

    // for sysid
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_int_distribution<int> distOdo;
    Vector2d odoOffset = Vector2d(0, 0);

public:  // Public Methods
    OdometrySubsystem(src::Drivers* drivers, tap::motor::DjiMotor* odo);

    //~OdometrySubsystem() {}  // Intentionally left blank

    void initialize();

    // getX, getY, their velocities and relocalizeTo are in the field frame (REP-105,
    // (0, 0) at the centre, x toward blue's base, OdometrySubsystemConstants.hpp), what the
    // Jetson speaks. The pods are x right, y forward of the heading at power-on, which is
    // the team's start.

    // gives x accounting for relocalize offset
    float getX();
    
    // gives y accounting for relocalize offset
    float getY();
    
    float getXVel();
    float getYVel();

    // sets the offsets so that if the odo pods don't move after you call this, getX and getY return newX and newY
    void relocalizeTo(float newX, float newY);

    // field yaw of the heading at power-on (the pods' and the IMU's zero): 0 red, PI blue
    float getStartYaw();

    // the field point forward and left of the start, along the heading at power-on
    Vector2d fromStart(float forward, float left);
    
    void refresh() override;

    /*
     * tells the motor to move the odometry to its specified angle calculated in update();
     */
    void updateMotor(float targetOdo, float odoAngleRelativeWorld, float odoVelRelativeWorld, float driveTrainAngularVelocity);

    void stopMotors();

    float getOdoEncoderValue();

    float getOdoVel();

private:  // Private Methods
    int getOdoVoltage(float driveTrainAngularVelocity, float odoAngleRelativeWorld, float odoAngularVelocity, float desiredAngleWorld, float inputVel, float dt);

    // gets the actual encoder value x. doesn't change after relocalize
    float getRawX();
    // gets the actual encoder value y. doesn't change after relocalize
    float getRawY();

};
}  // namespace subsystems