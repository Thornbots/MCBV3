#include "FlywheelSubsystem.hpp"

namespace subsystems {
    FlywheelSubsystem::FlywheelSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* flywheel1, tap::motor::DjiMotor* flywheel2) 
      : tap::control::Subsystem(drivers),
        motorFlywheel1(flywheel1),
        motorFlywheel2(flywheel2),
        drivers(drivers) {} 

        /**
         * initializes the 2 flywheels
         */
    void FlywheelSubsystem::initialize() {
        drivers->commandScheduler.registerSubsystem(this);

        motorFlywheel1->initialize();
        motorFlywheel2->initialize();
    }
    
    /**
     * sets the target motor RPM of the flywheels
     */
    void FlywheelSubsystem::setTargetVelocity(int targetMotorRPM){
        this->targetMotorRPM = targetMotorRPM;
        flywheelPIDController1.runControllerDerivateError(targetMotorRPM - motorFlywheel1->getShaftRPM(), 1);
        flywheelPIDController2.runControllerDerivateError(targetMotorRPM - motorFlywheel2->getShaftRPM(), 1);

        flyWheel1Voltage = static_cast<int32_t>(flywheelPIDController1.getOutput());
        flyWheel2Voltage = static_cast<int32_t>(flywheelPIDController2.getOutput());
    }

    /**
     * refreshes flywheel output
     */
    void FlywheelSubsystem::refresh() {
        motorFlywheel1->setDesiredOutput(flyWheel1Voltage);
        motorFlywheel2->setDesiredOutput(flyWheel2Voltage);
    }

    /**
     * turn off
     */
    void FlywheelSubsystem::stopMotors() {
        flyWheel1Voltage = 0;
        flyWheel2Voltage = 0;
    }

    float FlywheelSubsystem::getShootingVelocity() {
        return targetMotorRPM * 2 * (FLYWHEEL_RADIUS_MM / 1000.0f) * PI / 60.0f;
    }

}  // namespace subsystems