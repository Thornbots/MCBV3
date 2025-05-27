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

void DoubleIndexerSubsystem::indexAtRate(float ballsPerSecond){
    ballsPerSecond = ballsPerSecond/2;
    IndexerSubsystem::indexAtRate(ballsPerSecond);

    // Check if the firing rate should be limited to prevent overheating
    tap::communication::serial::RefSerial::Rx::TurretData turretData = drivers->refSerial.getRobotData().turret;
    if (drivers->refSerial.getRefSerialReceivingData() && (HEAT_PER_BALL * ballsPerSecond - turretData.coolingRate) * LATENCY > (turretData.heatLimit - turretData.heat17ID2)) {
        ballsPerSecond = turretData.coolingRate / HEAT_PER_BALL;
    }

    setTargetMotor2RPM(ballsPerSecond * 60.0f * REV_PER_BALL);
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
