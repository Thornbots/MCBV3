#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"
#include "tap/control/subsystem.hpp"

#include "FlywheelSubsystemConstants.hpp"
#include "drivers.hpp"

namespace subsystems
{

static tap::arch::PeriodicMilliTimer shooterControllerTimer(2);
static tap::arch::PeriodicMilliTimer secondTimer(100);
class FlywheelSubsystem : public tap::control::Subsystem
{
public:  // Public Variables
// constexpr static float PI = 3.14159;

private:  // Private Variables
src::Drivers* drivers;

tap::motor::DjiMotor* motorFlywheel1;
tap::motor::DjiMotor* motorFlywheel2;

tap::algorithms::SmoothPid flywheelPIDController1{PID_CONF_FLYWHEEL};
tap::algorithms::SmoothPid flywheelPIDController2{PID_CONF_FLYWHEEL};

int32_t flyWheel1Voltage = 0;
int32_t flyWheel2Voltage = 0;

int targetMotorRPM;

bool robotDisabled = false;

public:  // Public Methods 
    FlywheelSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* flywheel1, tap::motor::DjiMotor* flywheel2);

    ~FlywheelSubsystem() {}  // Intentionally left blank

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

    void setTargetVelocity(int targetMotorRPM);
    /*
        * Call this function to set all Turret motors to 0 desired RPM, calculate the voltage level in which to achieve this quickly
        * and packages this information for the motors TO BE SENT over CanBus
        */
    void stopMotors();

    float getShootingVelocity();

    private:  // Private Methods
    };
}  // namespace ThornBots
