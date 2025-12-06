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
    revPerBall(revPerBall/GEAR_RATIO),
    doPositionControl(doPositionControl),
    homingState(doHoming ? HomingState::NEED_TO_HOME : HomingState::DONT_HOME)
    {}

void IndexerSubsystem::initialize() {
    motorIndexer->initialize();
    drivers->commandScheduler.registerSubsystem(this);

}

void IndexerSubsystem::refresh() {    
    doHomingTransitions();
    if (homingState==HomingState::HOMING) {
        homeIndexer();
    }
    
    // if (!drivers->refSerial.getRefSerialReceivingData() || drivers->refSerial.getRobotData().robotPower&RefSerialData::Rx::RobotPower::SHOOTER_HAS_POWER)
    // if(ballsPerSecond==0){
    //     motorIndexer->setDesiredOutput(0);
    //     indexerController.clearBuildup();
    // } else {
    if(drivers->remote.isConnected()) {
        // do velocity control when unjamming or homing (temporaryVelocityControl) or we want velocity control (!doPositionControl)
        if(temporaryVelocityControl || !doPositionControl){
            velocityControl();
        } else {
            positionControl();
        }
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
    temporaryVelocityControl = false;
    
    if(doAutoUnjam(inputBallsPerSecond)) {
        unjam(true);
        return this->ballsPerSecond;
    }
    
    this->ballsPerSecond = counter.getAllowableIndexRate(inputBallsPerSecond);
    
    return this->ballsPerSecond;
}


void IndexerSubsystem::stopIndex() {
    ballsPerSecond = 0;
    temporaryVelocityControl = false;
}

void IndexerSubsystem::unjam(bool){
    temporaryVelocityControl = true;
    ballsPerSecond = UNJAM_BALL_PER_SECOND;
}


// converts delta motor ticks to num balls shot using constants
float IndexerSubsystem::getNumBallsShot() {
    return counter.getRecentNumBallsShot();
}

// position and velo in radians of the output (where spindex is attached) shaft
int IndexerSubsystem::getIndexerVoltage(float currentPosition, float currentVelocity, float targetPosition, float inputVelocity, float deltaT) {
    return 1000 * indexerController.calculate(currentPosition, currentVelocity, targetPosition, inputVelocity, deltaT); 
}

float IndexerSubsystem::getTotalNumBallsShot() {
    return counter.getTotalNumBallsShot();
}

void IndexerSubsystem::resetBallsCounter() {
    counter.resetRecentBallsCounter();
}

void IndexerSubsystem::incrementTargetNumBalls() {
    targetIndexerPosition+=getPositionIncrement(); 
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

bool IndexerSubsystem::refPoweringIndex() {
    return !drivers->refSerial.getRefSerialReceivingData() || drivers->refSerial.getRobotData().robotPower.any(RefSerialData::Rx::RobotPower::SHOOTER_HAS_POWER);
}

int32_t IndexerSubsystem::getEstHeat(){
    return counter.getEstHeat();
}
bool IndexerSubsystem::heatAllowsShooting(){
    return counter.canShootAgain();
}

bool IndexerSubsystem::canShoot() {
    return isIndexOnline() && refPoweringIndex() && heatAllowsShooting() && isProjectileAtBeam();
}


void IndexerSubsystem::doHomingTransitions(){
    // if was homed and went offline, need to home again
    if(!(isIndexOnline() && refPoweringIndex()) && homingState>=HomingState::HOMED){ //homed or gave up homing and the motor is offline
        homingState = HomingState::NEED_TO_HOME;
    }
    // if waiting to home, start it now if possible
    if(isIndexOnline() && refPoweringIndex() && homingState==HomingState::NEED_TO_HOME && drivers->recal.getIsImuReady() && drivers->remote.isConnected()){
        homingState=HomingState::HOMING;
        timeoutHome.restart(1000*HOMING_TIMEOUT);
        indexerController.clearBuildup();
    }
}

void IndexerSubsystem::homeIndexer() {
    temporaryVelocityControl = true;
    ballsPerSecond = HOMING_BALLS_PER_SECOND; 
    // needs to have a high torque for 100 consec cycles to be considered homed properly
    if (abs(motorIndexer->getTorque()) > 2000) {
        homingCounter++;
    } else {
        homingCounter = 0;
    }
    bool done = false;
    if (homingCounter >= 100) {
        done = true;
        homingState = HomingState::HOMED;
        targetIndexerPosition = getPositionIncrement()*INITIAL_INDEX_OFFSET;
    }
    if(timeoutHome.isExpired()){
        done = true;
        homingState = HomingState::GAVE_UP_HOMING;
        targetIndexerPosition = motorIndexer->getPositionUnwrapped()/GEAR_RATIO;
    }
    if(done){
        homingCounter = 0; //reset for next time
        temporaryVelocityControl = false;
        ballsPerSecond = 0;
        // indexNearest();
    }
}

void IndexerSubsystem::indexNearest() {
    //only applies to position control
    if(!doPositionControl) return;
    
    temporaryVelocityControl = false;
    float currentPos = motorIndexer->getPositionUnwrapped()/GEAR_RATIO;  //radians
    //index to the nearest next shot. This ensures we always have a shot ready to shoot
    targetIndexerPosition = (std::ceil((currentPos - getPositionIncrement()*INITIAL_INDEX_OFFSET) //find number of shots we are at
    /getPositionIncrement())
     + INITIAL_INDEX_OFFSET) //add the offset back
      * getPositionIncrement(); //convert to radians

}

float IndexerSubsystem::getCurrentOutputVelo() {
    return motorIndexer->getShaftRPM()*(PI/30)/GEAR_RATIO;
}

void IndexerSubsystem::positionControl(){
    motorIndexer->setDesiredOutput(getIndexerVoltage(motorIndexer->getPositionUnwrapped()/GEAR_RATIO, getCurrentOutputVelo(), targetIndexerPosition, 0, DT));
}

void IndexerSubsystem::velocityControl(){
    motorIndexer->setDesiredOutput(getIndexerVoltage(0, getCurrentOutputVelo(), 0, ballsPerSecond/revPerBall, DT)); //by giving it 0 target position and velo we effectively have a velo controller
}

float IndexerSubsystem::getPositionIncrement(){
    return 2*PI*revPerBall;
}
    

} //namespace subsystems 