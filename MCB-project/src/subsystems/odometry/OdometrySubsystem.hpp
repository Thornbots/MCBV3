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
    
    // changes as a result of relocalizeTo, used in getX and getY
    float offsetX = 0.0f;
    float offsetY = 0.0f;

    // for sysid
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_int_distribution<int> distOdo;
    Vector2d odoOffset = Vector2d(0, 0);

public:  // Public Methods
    OdometrySubsystem(src::Drivers* drivers, tap::motor::DjiMotor* odo);

    //~OdometrySubsystem() {}  // Intentionally left blank

    void initialize();

    // gives x accounting for relocalize offset
    float getX();
    
    // gives y accounting for relocalize offset
    float getY();
    
    float getXVel();
    float getYVel();

    // sets the offsets so that if the odo pods don't move after you call this, getX and getY return newX and newY
    void relocalizeTo(float newX, float newY);
    
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