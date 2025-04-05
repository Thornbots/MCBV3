#include "DoubleIndexerSubsystem.hpp"
#include "IndexerSubsystemConstants.hpp"

namespace subsystems
{

DoubleIndexerSubsystem::DoubleIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index1, tap::motor::DjiMotor* index2)
    : IndexerSubsystem(drivers, index1), // Call base class constructor
    motorIndexer2(index2),
    indexPIDController2(PID_CONF_INDEX)
{

    // Any additional initialization for the second motor, if necessary
}

void DoubleIndexerSubsystem::initialize() {
    IndexerSubsystem::initialize();
    // Initialize both motors
    motorIndexer2->initialize();     // Initialize the second motor
}

void DoubleIndexerSubsystem::refresh() {
    IndexerSubsystem::refresh();
    // Set the desired output for both motors
    motorIndexer2->setDesiredOutput(indexerVoltage2);   // Second motor (same voltage)
}

void DoubleIndexerSubsystem::stopIndex() {
    // Stop both motors
    IndexerSubsystem::stopIndex();
    indexerVoltage2 = 0;
}

void DoubleIndexerSubsystem::unjam(){
    indexAtRate(UNJAM_BALL_PER_SECOND);    
}

void DoubleIndexerSubsystem::indexAtRate(float ballsPerSecond){
    this->ballsPerSecond = ballsPerSecond;
    //divided by 2 by using 30.0f instead of 60.0f
    IndexerSubsystem::setTargetMotorRPM(ballsPerSecond * 30.0f * REV_PER_BALL);
    setTargetMotorRPM(ballsPerSecond * 30.0f * REV_PER_BALL);
}
void DoubleIndexerSubsystem::setTargetMotorRPM(int targetMotorRPM){

    indexPIDController2.runControllerDerivateError(targetMotorRPM - motorIndexer2->getShaftRPM(), 1);

    indexerVoltage2 = static_cast<int32_t>(indexPIDController2.getOutput());
}

} // namespace subsystems
