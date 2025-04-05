#pragma once
#include <random>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"

#include "controllers/OdometryController.hpp"

#include "drivers.hpp"

namespace subsystems {

class OdometrySubsystem : public tap::control::Subsystem {
private:  // Private Variables
    src::Drivers* drivers;
    tap::motor::DjiMotor* motorOdometry;
    OdometryController odometryController;  // default constructor

    float odometryMotorVoltage;

    float odometryEncoderCache = 0.0f;
    float driveTrainAngularVelocity = 0.0f;
    float targetOdometryAngleWorld = 0.0f;
    float driveTrainEncoder = 0.0f;
    float odometryAngleRelativeWorld = 0.0f;
    float odometryAngularVelocity = 0.0f;
    float imuOffset = 0.0f;

    // for sysid
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_int_distribution<int> distOdometry;

public:  // Public Methods
    OdometrySubsystem(src::Drivers* drivers, tap::motor::DjiMotor* yaw);

    ~OdometrySubsystem() {}

    void initialize();

    void refresh() override;

    /*
     * tells the motor to move the (upside down) gimbal to its specified angle
     */
    void updateMotor(float changeInTarget);

    /*
     * Call this function to set all Turret motors to stop, calculate the voltage level in
     * which to achieve this quickly and packages this information for the motors TO BE SENT over
     * CanBus
     */
    void stopMotors();

    float getOdometryEncoderValue();
    
    float getOdometryVel();

private:  // Private Methods
    int getOdometryVoltage(float driveTrainAngularVelocity, float odometryAngleRelativeWorld, float odometryAngularVelocity, float desiredAngleWorld, float inputVel, float dt);
};
}  // namespace subsystems