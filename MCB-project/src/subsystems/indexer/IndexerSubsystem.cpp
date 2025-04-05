#include "IndexerSubsystem.hpp"
#include "IndexerSubsystemConstants.hpp"

namespace subsystems {
    
IndexerSubsystem::IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index)
    : tap::control::Subsystem(drivers),
    drivers(drivers),
    motorIndexer(index),
    indexPIDController(PID_CONF_INDEX)
    {}

void IndexerSubsystem::initialize() {
    motorIndexer->initialize();
    drivers->commandScheduler.registerSubsystem(this);

}

void IndexerSubsystem::refresh() {
    motorIndexer->setDesiredOutput(indexerVoltage);
}

void IndexerSubsystem::indexAtRate(float ballsPerSecond) {
    // Check if the firing rate should be limited to prevent overheating
    tap::communication::serial::RefSerial::Rx::TurretData turretData = drivers->refSerial.getRobotData().turret;
    if (drivers->refSerial.getRefSerialReceivingData() && (HEAT_PER_BALL * ballsPerSecond - turretData.coolingRate) * LATENCY > (turretData.heatLimit - turretData.heat17ID1)) {
        ballsPerSecond = turretData.coolingRate / HEAT_PER_BALL;
    }

    this->ballsPerSecond = ballsPerSecond;
    setTargetMotorRPM(ballsPerSecond * 60.0f * REV_PER_BALL);
}

void IndexerSubsystem::indexAtMaxRate(){
    setTargetMotorRPM(MAX_INDEX_RPM);
}

void IndexerSubsystem::stopIndex() {
    // indexerVoltage = 0;
    indexAtRate(0);
}

void IndexerSubsystem::unjam(){
    indexAtRate(UNJAM_BALL_PER_SECOND);
}
//first 1800 degrees until first shot
//need to tell it to go max speed until we get to 1800 degrees from where we started
//so the first shot goes out asap

void IndexerSubsystem::setTargetMotorRPM(int targetMotorRPM) {
    indexPIDController.runControllerDerivateError(targetMotorRPM - motorIndexer->getShaftRPM(), 1);

    indexerVoltage = static_cast<int32_t>(indexPIDController.getOutput());
}

// converts delta motor ticks to num balls shot using constants
float IndexerSubsystem::getNumBallsShot() {
    return (motorIndexer->getPositionUnwrapped() - numTicksAtInit) / (REV_PER_BALL * M_2_PI);
}

void IndexerSubsystem::resetBallsCounter() {
    numTicksAtInit = motorIndexer->getPositionUnwrapped();
}

float IndexerSubsystem::getBallsPerSecond() {
    return ballsPerSecond;
}

} //namespace subsystems