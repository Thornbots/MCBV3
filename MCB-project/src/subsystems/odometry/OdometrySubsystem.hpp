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

    // Referee team selects route points; pod axes and boot heading stay fixed.
    bool isBlue = false;
    bool navigationLocalized = false;
    Vector2d aimPosition{0.0f, 0.0f};
    Vector2d lastPodPosition{0.0f, 0.0f};

    // for sysid
    std::mt19937 gen{std::mt19937::default_seed};
    std::uniform_int_distribution<int> distOdo;
    Vector2d odoOffset = Vector2d(0, 0);

public:  // Public Methods
    OdometrySubsystem(src::Drivers* drivers, tap::motor::DjiMotor* odo);

    //~OdometrySubsystem() {}  // Intentionally left blank

    void initialize();

    // getX/getY include the navigation offset replaced by relocalizeTo. Raw
    // getAimX/getAimY are the field-aligned pod coordinates sent in POSE.
    // Raw aim starts at zero; team selection never reanchors accumulated position.

    // gives x accounting for relocalize offset
    float getX();
    
    // gives y accounting for relocalize offset
    float getY();

    // Continuous field-aligned pod odometry for POSE and aim; never relocalized.
    float getAimX();
    float getAimY();
    
    float getXVel();
    float getYVel();

    // sets the offsets so that if the odo pods don't move after you call this, getX and getY return newX and newY
    void relocalizeTo(float newX, float newY);
    bool isNavigationLocalized() const { return navigationLocalized; }

    // Field yaw of the heading at power-on (the pods' and IMU's zero): PI/2 both teams.
    float getStartYaw();

    // Route basis points inward from each team's base, independently of chassis yaw.
    float getRouteYaw();

    // Field point inward and left from the team's start in the route basis.
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
    void updateAimPosition();
    int getOdoVoltage(float driveTrainAngularVelocity, float odoAngleRelativeWorld, float odoAngularVelocity, float desiredAngleWorld, float inputVel, float dt);

    // gets the actual encoder value x. doesn't change after relocalize
    float getRawX();
    // gets the actual encoder value y. doesn't change after relocalize
    float getRawY();

};
}  // namespace subsystems
