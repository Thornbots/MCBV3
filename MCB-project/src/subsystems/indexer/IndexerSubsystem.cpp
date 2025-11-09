#include "IndexerSubsystem.hpp"

namespace subsystems {
using namespace tap::communication::serial;

IndexerSubsystem::IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index, bool doHoming)
    : IndexerSubsystem(drivers, index, doHoming, ShotCounter::BarrelType::TURRET_17MM_EITHER){}

IndexerSubsystem::IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index, bool doHoming, ShotCounter::BarrelType barrel)
    : IndexerSubsystem(drivers, index, doHoming, barrel, REV_PER_BALL) {}

IndexerSubsystem::IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index, bool doHoming, ShotCounter::BarrelType barrel, float revPerBall)
    : tap::control::Subsystem(drivers),
    drivers(drivers),
    motorIndexer(index),
    indexPIDController(PID_CONF_INDEX),
    counter(drivers, barrel, index),
    revPerBall(revPerBall),
    homingState(doHoming ? HomingState::NEED_TO_HOME : HomingState::DONT_HOME)
    {}

void IndexerSubsystem::initialize() {
    motorIndexer->initialize();
    drivers->commandScheduler.registerSubsystem(this);

}

void IndexerSubsystem::refresh() {
    if(homingState>=HomingState::HOMED&&!motorIndexer->isMotorOnline()){ //homed or gave up homing and the motor is offline
        homingState = HomingState::NEED_TO_HOME;
    }
    if(homingState==HomingState::NEED_TO_HOME && motorIndexer->isMotorOnline() && drivers->recal.getIsImuReady()){
        homingState=HomingState::HOMING;
        timeoutHome.restart(1000*HOMING_TIMEOUT);
    }
    if (homingState==HomingState::HOMING) {
        homeIndexer();
    }
    // if (!drivers->refSerial.getRefSerialReceivingData() || drivers->refSerial.getRobotData().robotPower&RefSerialData::Rx::RobotPower::SHOOTER_HAS_POWER)
    motorIndexer->setDesiredOutput(indexerVoltage);
}


bool IndexerSubsystem::doAutoUnjam(float inputBallsPerSecond) {
    //if unjamming
    if(isAutoUnjamming){
        if(timeoutUnjam.isExpired()){
            timeoutUnjam.stop();
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
        if(timeoutUnjam.isStopped()){
            timeoutUnjam.restart(AUTO_UNJAM_TIME_UNDER_THRESH*1000);
        }

        if(timeoutUnjam.isExpired()){
            isAutoUnjamming = true;
            timeoutUnjam.restart(AUTO_UNJAM_TIME_UNJAMMING*1000);
            IndexerSubsystem::indexAtRate(UNJAM_BALL_PER_SECOND);
            return true;
        }
    }

    //for stopIndex
    if(inputBallsPerSecond==0){
        timeoutUnjam.stop();
        isAutoUnjamming = false;
    }

    return false;
}

float IndexerSubsystem::indexAtRate(float inputBallsPerSecond) {
    // if(homingState==HomingState::HOMING) return 0; //ignore if we are homing

    if(doAutoUnjam(inputBallsPerSecond)) return UNJAM_BALL_PER_SECOND;

    this->ballsPerSecond = counter.getAllowableIndexRate(inputBallsPerSecond);
    setTargetMotorRPM(this->ballsPerSecond * 60.0f * revPerBall);
    return this->ballsPerSecond;
}

void IndexerSubsystem::indexAtMaxRate(){
    // if(homingState==HomingState::HOMING) return; //ignore if we are homing

    setTargetMotorRPM(MAX_INDEX_RPM);
}

void IndexerSubsystem::stopIndex() {
    // do we set homing state to give up?

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

void IndexerSubsystem::incrementTargetNumBalls(int numBalls) {
    targetIndexerPosition+=counter.getPositionIncrement()*numBalls;
}

float IndexerSubsystem::getBallsPerSecond() {
    return ballsPerSecond;
}

float IndexerSubsystem::getActualBallsPerSecond() {
    return motorIndexer->getShaftRPM() / (60.0f * revPerBall);
}

bool IndexerSubsystem::isProjectileAtBeam() {
    return true;
}

bool IndexerSubsystem::isIndexOnline() {
    return motorIndexer->isMotorOnline();
}

void IndexerSubsystem::homeIndexer() {
    indexerVoltage = -2000; //make this a constant probably
    if (abs(motorIndexer->getTorque()) > 2000) { //same for this
        homingState = HomingState::HOMED;
        motorIndexer->resetEncoderValue();
    };
    if(timeoutHome.isExpired()){
        homingState = HomingState::GAVE_UP_HOMING;
    }
}

} //namespace subsystems