#include "IndexerSubsystem.hpp"
#include <cmath>

namespace subsystems {
using namespace tap::communication::serial;
using namespace subsystems::indexer;

IndexerSubsystem::IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index, bool doHoming, bool doPositionControl)
    : IndexerSubsystem(drivers, index, doHoming, doPositionControl, ShotCounter::BarrelType::TURRET_17MM_EITHER){}

IndexerSubsystem::IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index, bool doHoming, bool doPositionControl, ShotCounter::BarrelType barrel)
    : IndexerSubsystem(drivers, index, doHoming, doPositionControl, barrel, REV_PER_BALL) {}

IndexerSubsystem::IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index, bool doHoming, bool doPositionControl, ShotCounter::BarrelType barrel, float revPerBall)
    : tap::control::Subsystem(drivers),
    drivers(drivers),
    motorIndexer(index),
    indexPIDController(PID_CONF_INDEX),
    counter(drivers, barrel, index),
    revPerBall(revPerBall),
    doPositionControl(doPositionControl),
    homingState(doHoming ? HomingState::NEED_TO_HOME : HomingState::DONT_HOME)
    {}

void IndexerSubsystem::initialize() {
    motorIndexer->initialize();
    drivers->commandScheduler.registerSubsystem(this);

}

void IndexerSubsystem::refresh() {
    // need timer execute for ballspersecond
    if(homingState>=HomingState::HOMED&&!motorIndexer->isMotorOnline()){ //homed or gave up homing and the motor is offline
        homingState = HomingState::NEED_TO_HOME;
    }
    if(homingState==HomingState::NEED_TO_HOME && motorIndexer->isMotorOnline() && drivers->recal.getIsImuReady() && drivers->remote.isConnected()){
        homingState=HomingState::HOMING;
        timeoutHome.restart(1000*HOMING_TIMEOUT);
    }
    if (homingState==HomingState::HOMING) {
        homeIndexer();
    }
    // if (!drivers->refSerial.getRefSerialReceivingData() || drivers->refSerial.getRobotData().robotPower&RefSerialData::Rx::RobotPower::SHOOTER_HAS_POWER)
    // if(ballsPerSecond==0){
    //     motorIndexer->setDesiredOutput(0);
    //     indexerController.clearBuildup();
    // } else {
    if((homingState >= HomingState::HOMED || homingState == HomingState::DONT_HOME) && drivers->remote.isConnected()) { //only run positoin control when homed bc it fights homing sequence
        if(doPositionControl){
            motorIndexer->setDesiredOutput(getIndexerVoltage(motorIndexer->getPositionUnwrapped()/GEAR_RATIO, motorIndexer->getShaftRPM()*(PI/30)/GEAR_RATIO, targetIndexerPosition, 0, DT));
        } else {
            motorIndexer->setDesiredOutput(indexerVoltage);
        }
    } else if (drivers->remote.isConnected()) {
        motorIndexer->setDesiredOutput(getIndexerVoltage(0, motorIndexer->getShaftRPM()*(PI/30)/GEAR_RATIO, 0, -1.75, DT)); //by giving it 0 target position and velo we effectively have a velo controller
    } else {
        motorIndexer->setDesiredOutput(0); //disable indexer when remote is off
    }
    // }
    
    counter.update();
}


bool IndexerSubsystem::doAutoUnjam(float inputBallsPerSecond) {
    // figure out with position control later

    if(doPositionControl) return false;
    
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
    if(!doPositionControl) {
        if(doAutoUnjam(inputBallsPerSecond)) return UNJAM_BALL_PER_SECOND;
    }
    
    this->ballsPerSecond = counter.getAllowableIndexRate(inputBallsPerSecond); //repeatedly increment if it can shoot
    
    if(!doPositionControl) {
        setTargetMotorRPM(this->ballsPerSecond * 60.0f * revPerBall);
    }
    return this->ballsPerSecond;
}

void IndexerSubsystem::indexAtMaxRate(){
    // probably don't need anymore
    
    // if(homingState==HomingState::HOMING) return; //ignore if we are homing

    setTargetMotorRPM(MAX_INDEX_RPM);
}

void IndexerSubsystem::stopIndex() {
    // need to figure out soon

    if(doPositionControl){
        ballsPerSecond = 0;
    } else {
        indexAtRate(0);
    }
}

void IndexerSubsystem::unjam(){
    if(doPositionControl){
        motorIndexer->setDesiredOutput(getIndexerVoltage(0, motorIndexer->getShaftRPM()*(PI/30)/GEAR_RATIO, 0, -6, DT));
    } else {
        indexAtRate(UNJAM_BALL_PER_SECOND);
    }
}

// used for velo control (hero)
void IndexerSubsystem::setTargetMotorRPM(int targetMotorRPM) {
    indexPIDController.runControllerDerivateError(targetMotorRPM - motorIndexer->getShaftRPM(), 1);

    indexerVoltage = static_cast<int32_t>(indexPIDController.getOutput());
}

// converts delta motor ticks to num balls shot using constants
float IndexerSubsystem::getNumBallsShot() {
    return counter.getRecentNumBallsShot();
}

int IndexerSubsystem::getIndexerVoltage(float currentPosition, float currentVelocity, float targetPosition, float inputVelocity, float deltaT) {
    // return 10000;
    return 1000 * indexerController.calculate(currentPosition, currentVelocity, targetPosition, inputVelocity, 0.001f); //currentPosition, currentVelocity, targetPosition, inputVelocity, deltaT);
}

float IndexerSubsystem::getTotalNumBallsShot() {
    return counter.getTotalNumBallsShot();
}

void IndexerSubsystem::resetBallsCounter() {
    counter.resetRecentBallsCounter();
}

void IndexerSubsystem::incrementTargetNumBalls() {
    // if ( heatCounter <= drivers->refSerial.getRobotData().turret.heatLimit - 10) {
    shotTimingCounter = 0; //we just shot
    // heatCounter += 10;
    targetIndexerPosition+=counter.getPositionIncrement(); 
    counter.incrementTargetNumBalls();
}


float IndexerSubsystem::getBallsPerSecond() {
    return ballsPerSecond;
}

void IndexerSubsystem::setBallsPerSecond(float newBallsPerSecond){
    ballsPerSecond = newBallsPerSecond;
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

int32_t IndexerSubsystem::getEstHeat(){
    return counter.getEstHeat();
}
bool IndexerSubsystem::heatAllowsShooting(){
    return counter.canShootAgain();
}

bool IndexerSubsystem::canShoot() {
    return isIndexOnline() && heatAllowsShooting() && isProjectileAtBeam();
}

void IndexerSubsystem::homeIndexer() {
    if (abs(motorIndexer->getTorque()) > 2000) { //same for this
        homingCounter++;
       

    } else {
        homingCounter = 0;
    }
    if (homingCounter >= 100) {
        homingState = HomingState::HOMED;
        motorIndexer->resetEncoderValue();
        targetIndexerPosition = counter.getPositionIncrement()*INITIAL_INDEX_OFFSET;
    }
    if(timeoutHome.isExpired()){
        homingState = HomingState::GAVE_UP_HOMING;
        motorIndexer->resetEncoderValue();
        targetIndexerPosition = 0;
    }
}

void IndexerSubsystem::indexNearest() {
    float currentPos = motorIndexer->getPositionUnwrapped()/GEAR_RATIO;  //radians
    //index to the nearest next shot. This ensures we always have a shot ready to shoot
    targetIndexerPosition = (std::ceil((currentPos - counter.getPositionIncrement()*INITIAL_INDEX_OFFSET) //find number of shots we are at
    /counter.getPositionIncrement())
     + INITIAL_INDEX_OFFSET) //add the offset back
      * counter.getPositionIncrement(); //convert to radians

}

} //namespace subsystems 