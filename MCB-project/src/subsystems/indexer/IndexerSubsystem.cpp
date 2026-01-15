#include "IndexerSubsystem.hpp"
#include <cmath>

namespace subsystems {
using namespace tap::communication::serial;
// using namespace subsystems::indexer;



IndexerSubsystem::IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index)
    : tap::control::Subsystem(drivers),
    drivers(drivers),
    motorIndexer(index)
    // indexPIDController(PID_CONF_INDEX),
    // counter(drivers, barrel, index),
    // revPerBall(revPerBall/GEAR_RATIO),
    // homingState(doHoming ? HomingState::NEED_TO_HOME : HomingState::DONT_HOME)
    {}

void IndexerSubsystem::initialize() {
    drivers->commandScheduler.registerSubsystem(this);
    timer.stop();
    
    finishInitialize(); //initialize motors and any other things Single and Hero want to do
}

void IndexerSubsystem::refresh() {
    // not sure if this is helpful.
    // if(isHoming){
    //     isHoming = !shouldEndHoming();
    // } else {
    //     isHoming = shouldStartHoming();
    // }
    
    // check if indexAtRate or indexNShotsAtRate should call tryShootOnce
    if(timer.execute()) {
        if(numShotsRemaining==0){
            stopIndexingAtRate();
        } else {
            tryShootOnce();
            if(numShotsRemaining>0) numShotsRemaining--;
        }
    }
    
    finishRefresh();
}


bool IndexerSubsystem::indexAtRate(float inputShotsPerSecond) {
    if(inputShotsPerSecond<=0){
        stopIndexingAtRate();
        return false;
    }
    
    isManualUnjamming = false;
    isStopped = false;
    bool shootImmediately = isDoneIndexingAtRate();
    
    // just now indexing at rate, shoot once now
    if(shootImmediately){
        tryShootOnce();
    }
    
    // maybe this gets called repeatedly while someone is trying index.
    // so shouldn't do anything to the timer if there wasn't a change
    if(shotsPerSecond!=inputShotsPerSecond) {
        // set timer, shoot on execute
        timer.restart(1000/inputShotsPerSecond);
    }
    shotsPerSecond=inputShotsPerSecond;
    numShotsRemaining = -1;
    return shootImmediately;
}

bool IndexerSubsystem::indexNShotsAtRate(float inputShotsPerSecond, int numShots) {
    if(numShots<=0){
        stopIndexingAtRate();
        return false;
    } 
    bool shotImmediately = indexAtRate(inputShotsPerSecond);
    if(shotImmediately) 
        numShotsRemaining = numShots-1; //if shot one, count it
    else 
        numShotsRemaining = numShots;
            
    return shotImmediately;
}


void IndexerSubsystem::stopIndexingAtRate() {
    shotsPerSecond = 0;
    numShotsRemaining = 0;
    timer.stop();
    isManualUnjamming = false;
}

bool IndexerSubsystem::isDoneIndexingAtRate() {
    return timer.isStopped();
}

void IndexerSubsystem::stopIndex() {
    stopIndexingAtRate();
    isStopped = true;
    // maybe something else too
    finishStopIndex();
}

void IndexerSubsystem::idle() {
    stopIndexingAtRate();
    isStopped = false;
}


void IndexerSubsystem::manualUnjam() {
    stopIndexingAtRate();
    isStopped = false;
    isManualUnjamming = true;
}


// converts delta motor ticks to num balls shot using constants
// float IndexerSubsystem::getNumBallsShot() {
//     return counter.getRecentNumBallsShot();
// }

// position and velo in radians of the output (where spindex is attached) shaft
// int IndexerSubsystem::getIndexerVoltage(float currentPosition, float currentVelocity, float targetPosition, float inputVelocity, float deltaT) {
//     return 1000 * indexerController.calculate(currentPosition, currentVelocity, targetPosition, inputVelocity, deltaT); 
// }

// float IndexerSubsystem::getTotalNumBallsShot() {
//     return counter.getTotalNumBallsShot();
// }

// void IndexerSubsystem::resetBallsCounter() {
//     counter.resetRecentBallsCounter();
// }

// void IndexerSubsystem::incrementTargetNumBalls() {
//     targetIndexerPosition+=getPositionIncrement(); 
//     counter.incrementTargetNumBalls();
// }


// float IndexerSubsystem::getBallsPerSecond() {
//     return ballsPerSecond;
// }

// void IndexerSubsystem::setBallsPerSecond(float newBallsPerSecond){
//     ballsPerSecond = newBallsPerSecond;
// }

// float IndexerSubsystem::getActualBallsPerSecond() {
//     return motorIndexer->getShaftRPM() / (60.0f * revPerBall);
// }

// Hero overrides
bool IndexerSubsystem::isProjectileAtBeam() {
    return true;
}

bool IndexerSubsystem::isIndexOnline() {
    return motorIndexer->isMotorOnline();
}

bool IndexerSubsystem::refPoweringIndex() {
    return !drivers->refSerial.getRefSerialReceivingData() || drivers->refSerial.getRobotData().robotPower.any(RefSerialData::Rx::RobotPower::SHOOTER_HAS_POWER);
}

// int32_t IndexerSubsystem::getEstHeat(){
//     return counter.getEstHeat();
// }
// bool IndexerSubsystem::heatAllowsShooting(){
//     return counter.canShootAgain();
// }

bool IndexerSubsystem::canShoot() {
    // return true;
    return isIndexOnline() && refPoweringIndex() && heatAllowsShooting() && isProjectileAtBeam() && tap::arch::clock::getTimeMilliseconds()-lastShotTime>MIN_SHOT_FREQ;
}


// void IndexerSubsystem::doHomingTransitions(){
//     // if was homed and went offline, need to home again
//     if(!(isIndexOnline() && refPoweringIndex()) && homingState>=HomingState::HOMED){ //homed or gave up homing and the motor is offline
//         homingState = HomingState::NEED_TO_HOME;
//     }
//     // if waiting to home, start it now if possible
//     if(isIndexOnline() && refPoweringIndex() && homingState==HomingState::NEED_TO_HOME && drivers->recal.getIsImuReady() && drivers->remote.isConnected()){
//         homingState=HomingState::HOMING;
//         timeoutHome.restart(1000*HOMING_TIMEOUT);
//         indexerController.clearBuildup();
//     }
// }

// void IndexerSubsystem::homeIndexer() {
//     temporaryVelocityControl = true;
//     ballsPerSecond = HOMING_BALLS_PER_SECOND; 
//     // needs to have a high torque for 100 consec cycles to be considered homed properly
//     if (abs(motorIndexer->getTorque()) > 2000) {
//         homingCounter++;
//     } else {
//         homingCounter = 0;
//     }
//     bool done = false;
//     if (homingCounter >= 100) {
//         done = true;
//         homingState = HomingState::HOMED;
//         targetIndexerPosition = getPositionIncrement()*INITIAL_INDEX_OFFSET;
//     }
//     if(timeoutHome.isExpired()){
//         done = true;
//         homingState = HomingState::GAVE_UP_HOMING;
//         targetIndexerPosition = motorIndexer->getPositionUnwrapped()/GEAR_RATIO;
//     }
//     if(done){
//         homingCounter = 0; //reset for next time
//         temporaryVelocityControl = false;
//         ballsPerSecond = 0;
//         // indexNearest();
//     }
// }

// void IndexerSubsystem::indexNearest() {
//     //only applies to position control
//     if(!doPositionControl) return;
    
//     temporaryVelocityControl = false;
//     float currentPos = motorIndexer->getPositionUnwrapped()/GEAR_RATIO;  //radians
//     //index to the nearest next shot. This ensures we always have a shot ready to shoot
//     targetIndexerPosition = (std::ceil((currentPos - getPositionIncrement()*INITIAL_INDEX_OFFSET) //find number of shots we are at
//     /getPositionIncrement())
//      + INITIAL_INDEX_OFFSET) //add the offset back
//       * getPositionIncrement(); //convert to radians

// }

// float IndexerSubsystem::getCurrentOutputVelo() {
//     return motorIndexer->getShaftRPM()*(PI/30)/GEAR_RATIO;
// }

// void IndexerSubsystem::positionControl(){
//     motorIndexer->setDesiredOutput(getIndexerVoltage(motorIndexer->getPositionUnwrapped()/GEAR_RATIO, getCurrentOutputVelo(), targetIndexerPosition, 0, DT));
// }

// void IndexerSubsystem::velocityControl(){
//     motorIndexer->setDesiredOutput(getIndexerVoltage(0, getCurrentOutputVelo(), 0, ballsPerSecond/revPerBall, DT)); //by giving it 0 target position and velo we effectively have a velo controller
// }

// float IndexerSubsystem::getPositionIncrement(){
//     return 2*PI*revPerBall;
// }

void IndexerSubsystem::justShot() {
    lastShotTime = tap::arch::clock::getTimeMilliseconds();
}
    

} //namespace subsystems 