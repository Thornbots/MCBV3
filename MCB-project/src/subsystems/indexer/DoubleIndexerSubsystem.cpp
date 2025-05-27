#include "DoubleIndexerSubsystem.hpp"

namespace subsystems
{

DoubleIndexerSubsystem::DoubleIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index1, tap::motor::DjiMotor* index2)
    : IndexerSubsystem(drivers, index1), // Call base class constructor
    motorIndexer2(index2),
    indexPIDController2(PID_CONF_INDEX),
    counter2(drivers, ShotCounter::BarrelType::TURRET_17MM_2, index2)
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

float DoubleIndexerSubsystem::indexAtRate(float ballsPerSecond){
    //first barrel
    float ballsPerSecond1 = ballsPerSecond/2;
    IndexerSubsystem::indexAtRate(ballsPerSecond1);

    //second barrel
    float ballsPerSecond2 = counter.getAllowableIndexRate(ballsPerSecond/2);
    setTargetMotor2RPM(ballsPerSecond2 * 60.0f * REV_PER_BALL);

    this->ballsPerSecond = ballsPerSecond1+ballsPerSecond2;
    return this->ballsPerSecond;
}

void DoubleIndexerSubsystem::indexAtMaxRate(){
    IndexerSubsystem::indexAtMaxRate();
    setTargetMotor2RPM(MAX_INDEX_RPM);
}

void DoubleIndexerSubsystem::setTargetMotor2RPM(int targetMotorRPM){

    indexPIDController2.runControllerDerivateError(targetMotorRPM - motorIndexer2->getShaftRPM(), 1);

    indexerVoltage2 = static_cast<int32_t>(indexPIDController2.getOutput());
}

} // namespace subsystems
