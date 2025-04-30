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




namespace subsystems
{

class OdometrySubsystem : public tap::control::Subsystem
{

private:  // Private Variables
    src::Drivers* drivers;
    // TODO: Check all motor ID's, and verify indexers and flywheels are in the correct direction
    tap::motor::DjiMotor* motorOdo;

    OdoController odoController;  // default constructor

    float odoMotorVoltage, driveTrainEncoder, odoEncoderCache, driveTrainAngularVelocity, odoAngleRelativeWorld, odoAngularVelocity;

    static constexpr float targetOdoAngleWorld = 0;

    // for sysid
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_int_distribution<int> distOdo;

public:  // Public Methods
    OdometrySubsystem(src::Drivers* drivers, tap::motor::DjiMotor* odo);

    //~OdometrySubsystem() {}  // Intentionally left blank

    /*
     * Call this function once, outside of the main loop.
     * This function will initalize all of the motors, timers, pidControllers, and any other used
     * object. If you want to know what initializing actually does, ping Teaney in discord, or just
     * Google it. It's pretty cool.
     */
    void initialize();

    /*
     * reads the right joystick values and updates the internal values of where the gimbal needs to
     * go
     */
    void refresh() override;

    /*
     * tells the motor to move the odometry to its specified angle calculated in update();
     */
    void updateMotor(float targetOdo, float odoAngleRelativeWorld, float odoVelRelativeWorld, float driveTrainAngularVelocity);

    /*
     * Call this function to set all Turret motors to stop, calculate the voltage level in
     * which to achieve this quickly and packages this information for the motors TO BE SENT over
     * CanBus
     */
    void stopMotors();

    float getOdoEncoderValue();

    float getOdoVel();

private:  // Private Methods
    int getOdoVoltage(float driveTrainAngularVelocity, float odoAngleRelativeWorld, float odoAngularVelocity, float desiredAngleWorld, float inputVel, float dt);
};
}  // namespace subsystems