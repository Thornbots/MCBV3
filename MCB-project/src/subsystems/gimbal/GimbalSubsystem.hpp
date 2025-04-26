#pragma once
#include <random>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

#include "controllers/PitchController.hpp"
#include "controllers/YawController.hpp"
#include "drivers.hpp"




namespace subsystems
{

static tap::arch::PeriodicMilliTimer turretControllerTimer(2);
class GimbalSubsystem : public tap::control::Subsystem
{

private:  // Private Variables
    src::Drivers* drivers;
    // TODO: Check all motor ID's, and verify indexers and flywheels are in the correct direction
    tap::motor::DjiMotor* motorYaw;
    tap::motor::DjiMotor* motorPitch;

    YawController yawController;  // default constructor
    PitchController pitchController;

    float pitchMotorVoltage, yawMotorVoltage;

    float driveTrainAngularVelocity, yawAngularVelocity, yawAngleRelativeWorld = 0.0, imuOffset;
    float gimbalPitchAngleRelativeWorld, gimbalPitchAngularVelocity;
    float yawEncoderCache = 0;
    float desiredYawAngleWorld, desiredYawAngleWorld2, driveTrainEncoder = 0.0;
    float stickAccumulator = 0, targetYawAngleWorld = 0,
          targetDTVelocityWorld = 0;  

    // for sysid
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_int_distribution<int> distYaw;
    std::uniform_int_distribution<int> distPitch;

public:  // Public Methods
    GimbalSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* yaw, tap::motor::DjiMotor* pitch);

    //~GimbalSubsystem() {}  // Intentionally left blank

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
     * tells the motors to move the gimbal to its specified angle calculated in update();
     */
    void updateMotors(float changeInTargetYaw, float targetPitch);

    /*
     *   Straight up gives a full state for the controllers to target
     */

    void updateMotorsAndVelocity(float changeInTargetYaw, float targetPitch, float targetYawVel, float targetPitchVel);
    /*
     * Call this function to set all Turret motors to stop, calculate the voltage level in
     * which to achieve this quickly and packages this information for the motors TO BE SENT over
     * CanBus
     */
    void stopMotors();

    /*
     * Call this function (any number of times) to reZero the yaw motor location. This will be used
     * when first turning on the robot and setting the Turret to where the front of the DriveTrain
     * is. This function should be called when either in the bootup sequence, or when some,
     * undetermined button is pressed on the keyboard.
     */
    void reZeroYaw();

    float getYawEncoderValue();

    float getPitchEncoderValue();

    float getYawVel();
    float getPitchVel();

    float getYawAngleRelativeWorld();

private:  // Private Methods
    int getPitchVoltage(float targetAngle, float pitchAngleRelativeGimbal, float pitchAngularVelocity, float dt);
    int getYawVoltage(float driveTrainAngularVelocity, float yawAngleRelativeWorld, float yawAngularVelocity, float desiredAngleWorld, float inputVel, float dt);
};
}  // namespace subsystems