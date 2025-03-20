#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "ChassisController.hpp"

#include "drivers.hpp"

namespace subsystems
{

class DrivetrainSubsystem : public tap::control::Subsystem
{

private:                                            // Private Variables
    tap::Drivers* drivers;

    tap::motor::DjiMotor* motorArray[4];

    float powerLimit = 100; //default value

    Pose2d lastDrive;


    float motorCurrent[4] = {0.0f,0.0f,0.0f,0.0f};

    float motorVel[4] = {0.0f,0.0f,0.0f,0.0f};

    ChassisController controller;
   

public:  // Public Methods
    DrivetrainSubsystem(tap::Drivers* driver, tap::motor::DjiMotor* motorOne, tap::motor::DjiMotor* motorTwo, tap::motor::DjiMotor* motorThree, tap::motor::DjiMotor* motorFour);
    
    // ~DrivetrainSubsystem() override {}  // Intentionally blank

    float imuAngle;

    /*
     * Call this function once, outside of the main loop.
     * This function will initalize all of the motors, timers, pidControllers, and any other used
     * object. If you want to know what initializing actually does, ping Teaney in discord, or just
     * Google it. It's pretty cool.
     */
    void initialize();

    void refresh() override;
    /*
     * Call this function when you want the Turret to follow the DriveTrain
     * Should be called within the main loop, so called every time in the main loop when you want
     * the described behavior. This will allow the drivetrain to translate with left stick, and turn
     * with the right stick or beyblade depending how this is called.
     */
    void setTargetTranslation(Pose2d drive);

    /*
     * Call this function to set all DriveTrain motors to 0 desired RPM. CALL setMotorSpeeds() FOR
     * THIS TO WORK
     */
    void stopMotors();

private:  // Private Methods
    

};
}  // namespace subsystems