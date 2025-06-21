#include "IndexerSubsystem.hpp"

namespace subsystems {
using namespace tap::communication::serial;
    
IndexerSubsystem::IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index)
    : IndexerSubsystem(drivers, index, ShotCounter::BarrelType::TURRET_17MM_1){}

IndexerSubsystem::IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index, ShotCounter::BarrelType barrel)
    : IndexerSubsystem(drivers, index, barrel, REV_PER_BALL) {}

IndexerSubsystem::IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index, ShotCounter::BarrelType barrel, float revPerBall)
    : tap::control::Subsystem(drivers),
    drivers(drivers),
    motorIndexer(index),
    indexPIDController(PID_CONF_INDEX),
    counter(drivers, barrel, index),
    revPerBall(revPerBall)
    {}

void IndexerSubsystem::initialize() {
    motorIndexer->initialize();
    drivers->commandScheduler.registerSubsystem(this);

}

void IndexerSubsystem::refresh() {
    if (!drivers->refSerial.getRefSerialReceivingData() || drivers->refSerial.getRobotData().robotPower&RefSerialData::Rx::RobotPower::SHOOTER_HAS_POWER)
    {
        motorIndexer->setDesiredOutput(indexerVoltage);
    }
}


bool IndexerSubsystem::doAutoUnjam(float inputBallsPerSecond) {
    //if unjamming
    if(isAutoUnjamming){
        if(timeout.isExpired()){
            timeout.stop();
            isAutoUnjamming = false;
        } else if (inputBallsPerSecond > 0) {
            //prevent infinite recursion, unjam calls indexAtRate with a negative number
            IndexerSubsystem::indexAtRate(UNJAM_BALL_PER_SECOND);
            return true;
        }
    }
    
    //if we are here, isAutoUnjamming is false or inputBallsPerSecond<=0
    //if we are slow and trying to go forward
    if(inputBallsPerSecond > 0 && getActualBallsPerSecond()<AUTO_UNJAM_BALLS_PER_SEC_THRESH){
        if(timeout.isStopped()){
            timeout.restart(AUTO_UNJAM_TIME_UNDER_THRESH*1000);
        }

        if(timeout.isExpired()){
            isAutoUnjamming = true;
            timeout.restart(AUTO_UNJAM_TIME_UNJAMMING*1000);
            IndexerSubsystem::indexAtRate(UNJAM_BALL_PER_SECOND);
            return true;
        }
    }

    //for stopIndex
    if(inputBallsPerSecond==0){
        timeout.stop();
        isAutoUnjamming = false;
    }

    return false;
}

float IndexerSubsystem::indexAtRate(float inputBallsPerSecond) {
    if(doAutoUnjam(inputBallsPerSecond)) return UNJAM_BALL_PER_SECOND;

    this->ballsPerSecond = counter.getAllowableIndexRate(inputBallsPerSecond);
    setTargetMotorRPM(this->ballsPerSecond * 60.0f * revPerBall);
    return this->ballsPerSecond;
}

void IndexerSubsystem::indexAtMaxRate(){
    setTargetMotorRPM(MAX_INDEX_RPM);
}

void IndexerSubsystem::stopIndex() {
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
    return counter.getRecentNumBallsShot();
}

float IndexerSubsystem::getTotalNumBallsShot() {
    return counter.getTotalNumBallsShot();
}

void IndexerSubsystem::resetBallsCounter() {
    counter.resetRecentBallsCounter();
}

float IndexerSubsystem::getBallsPerSecond() {
    return ballsPerSecond;
}

float IndexerSubsystem::getActualBallsPerSecond() {
    return motorIndexer->getShaftRPM() / (60.0f * revPerBall);
}

} //namespace subsystems