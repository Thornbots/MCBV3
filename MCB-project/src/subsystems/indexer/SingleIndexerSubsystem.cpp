#include "SingleIndexerSubsystem.hpp"
#include <cmath>

namespace subsystems {
using namespace tap::communication::serial;
using namespace subsystems::indexer;

SingleIndexerSubsystem::SingleIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index)
    : IndexerSubsystem(drivers, index),
    unit(drivers, index, REV_PER_BALL/GEAR_RATIO, REV_PER_BALL),
    counter(drivers, ShotCounter::BarrelType::TURRET_17MM_EITHER, index),
    homingState(HomingState::NEED_TO_HOME)
    {}

void SingleIndexerSubsystem::finishInitialize() {
    unit.initialize();
}

void SingleIndexerSubsystem::finishRefresh() {
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
        // if(temporaryVelocityControl || !doPositionControl){
        //     velocityControl();
        // } else {
        //     positionControl();
        // }
        if(homingState>=HomingState::HOMED) { //if homed or gave up
            // allow unjam only if homed. Might be a bad idea.
            if(isManualUnjamming){
                shouldIndexNearest = true; //when we stop unjamming, index nearest
                unit.velocityControl(UNJAM_BALL_PER_SECOND);
            } else {
                if(!isStopped&&shouldIndexNearest){
                    unit.indexNearest();
                    shouldIndexNearest = false; //index nearest only once
                }
                unit.positionControl();
            }
        }
        // otherwise homeIndexer would have set velocity control
    } else {
        motorIndexer->setDesiredOutput(0); //disable indexer when remote is off
    }
    // }
    
    counter.update();
}


// bool SingleIndexerSubsystem::doAutoUnjam(float inputBallsPerSecond) {
//     // figure out with position control later

//     if(doPositionControl) return false;
    
//     //if unjamming
//     if(isAutoUnjamming){
//         if(timeoutUnjam.isExpired()){
//             timeoutUnjam.stop();
//             isAutoUnjamming = false;
//         } else if (inputBallsPerSecond > 0) {
//             return true;
//         }
//     }

//     //if we are here, isAutoUnjamming is false or inputBallsPerSecond<=0
//     //if we are slow and trying to go forward
//     if(inputBallsPerSecond > 0 && getActualBallsPerSecond()<AUTO_UNJAM_BALLS_PER_SEC_THRESH){
//         if(timeoutUnjam.isStopped()){
//             timeoutUnjam.restart(AUTO_UNJAM_TIME_UNDER_THRESH*1000);
//         }

//         if(timeoutUnjam.isExpired()){
//             isAutoUnjamming = true;
//             timeoutUnjam.restart(AUTO_UNJAM_TIME_UNJAMMING*1000);
//             return true;
//         }
//     }

//     //for stopIndex
//     if(inputBallsPerSecond==0){
//         timeoutUnjam.stop();
//         isAutoUnjamming = false;
//     }

//     return false;
// }

// float SingleIndexerSubsystem::indexAtRate(float inputBallsPerSecond) {
//     temporaryVelocityControl = false;
    
//     if(doAutoUnjam(inputBallsPerSecond)) {
//         unjam(true);
//         return this->ballsPerSecond;
//     }
    
//     this->ballsPerSecond = counter.getAllowableIndexRate(inputBallsPerSecond);
    
//     return this->ballsPerSecond;
// }


void SingleIndexerSubsystem::finishStopIndex() {
    // ballsPerSecond = 0;
    temporaryVelocityControl = false;
    shouldIndexNearest = true;
}

// void SingleIndexerSubsystem::unjam(bool){
//     temporaryVelocityControl = true;
//     ballsPerSecond = UNJAM_BALL_PER_SECOND;
// }


// converts delta motor ticks to num balls shot using constants
// float SingleIndexerSubsystem::getNumBallsShot() {
//     return counter.getRecentNumBallsShot();
// }

// int SingleIndexerSubsystem::getIndexerVoltage(float currentPosition, float currentVelocity, float targetPosition, float inputVelocity, float deltaT) {
//     return 1000 * indexerController.calculate(currentPosition, currentVelocity, targetPosition, inputVelocity, deltaT); 
// }

// float SingleIndexerSubsystem::getTotalNumBallsShot() {
//     return counter.getTotalNumBallsShot();
// }

// void SingleIndexerSubsystem::resetBallsCounter() {
//     counter.resetRecentBallsCounter();
// }

void SingleIndexerSubsystem::forceShootOnce() {
    // targetIndexerPosition+=getPositionIncrement(); 
    unit.shootOnce();
    counter.incrementTargetNumBalls();
    justShot();
}

bool SingleIndexerSubsystem::tryShootOnce() {
    bool r = canShoot();
    if(r) forceShootOnce();
    return r;
}


// float SingleIndexerSubsystem::getBallsPerSecond() {
//     return ballsPerSecond;
// }

// void SingleIndexerSubsystem::setBallsPerSecond(float newBallsPerSecond){
//     ballsPerSecond = newBallsPerSecond;
// }

// float SingleIndexerSubsystem::getActualBallsPerSecond() {
//     return motorIndexer->getShaftRPM() / (60.0f * revPerBall);
// }

// bool SingleIndexerSubsystem::isProjectileAtBeam() {
//     return true;
// }

// bool SingleIndexerSubsystem::isIndexOnline() {
//     return motorIndexer->isMotorOnline();
// }

// bool SingleIndexerSubsystem::refPoweringIndex() {
//     return !drivers->refSerial.getRefSerialReceivingData() || drivers->refSerial.getRobotData().robotPower.any(RefSerialData::Rx::RobotPower::SHOOTER_HAS_POWER);
// }

int32_t SingleIndexerSubsystem::getEstHeat(){
    return counter.getEstHeat();
}
float SingleIndexerSubsystem::getEstHeatPercentage(){
    if(drivers->refSerial.getRefSerialReceivingData())
        return counter.getEstHeat()/drivers->refSerial.getRobotData().turret.heatLimit;
    return 0;
}
bool SingleIndexerSubsystem::heatAllowsShooting(){
    return counter.canShootAgain();
}

float SingleIndexerSubsystem::getTotalNumBallsShot(){
    return counter.getTotalNumBallsShot();
}


void SingleIndexerSubsystem::doHomingTransitions(){
    // if was homed and went offline, need to home again
    if(!(isIndexOnline() && refPoweringIndex()) && homingState>=HomingState::HOMED){ //homed or gave up homing and the motor is offline
        homingState = HomingState::NEED_TO_HOME;
    }
    // if waiting to home, start it now if possible
    if(isIndexOnline() && refPoweringIndex() && homingState==HomingState::NEED_TO_HOME && drivers->recal.getIsImuReady() && drivers->remote.isConnected()){
        homingState=HomingState::HOMING;
        timeoutHome.restart(1000*HOMING_TIMEOUT);
        unit.clearBuildup();
    }
}

void SingleIndexerSubsystem::homeIndexer() {
    temporaryVelocityControl = true;
    // ballsPerSecond = HOMING_BALLS_PER_SECOND; 
    unit.velocityControl(HOMING_BALLS_PER_SECOND);
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
        // targetIndexerPosition = getPositionIncrement()*INITIAL_INDEX_OFFSET;
        unit.doAfterHomingOffset();
    }
    if(timeoutHome.isExpired()){
        done = true;
        homingState = HomingState::GAVE_UP_HOMING;
        // targetIndexerPosition = motorIndexer->getPositionUnwrapped()/GEAR_RATIO;
        unit.doGiveUpHomingOffset();
    }
    if(done){
        homingCounter = 0; //reset for next time
        temporaryVelocityControl = false;
        // ballsPerSecond = 0;
        unit.velocityControl(0);
        // indexNearest();
    }
}

void SingleIndexerSubsystem::indexNearest() {
    unit.indexNearest();
}

// float SingleIndexerSubsystem::getCurrentOutputVelo() {
//     return motorIndexer->getShaftRPM()*(PI/30)/GEAR_RATIO;
// }

// void SingleIndexerSubsystem::positionControl(){
//     motorIndexer->setDesiredOutput(getIndexerVoltage(motorIndexer->getPositionUnwrapped()/GEAR_RATIO, getCurrentOutputVelo(), targetIndexerPosition, 0, DT));
// }

// void SingleIndexerSubsystem::velocityControl(){
//     motorIndexer->setDesiredOutput(getIndexerVoltage(0, getCurrentOutputVelo(), 0, ballsPerSecond/revPerBall, DT)); //by giving it 0 target position and velo we effectively have a velo controller
// }

// float SingleIndexerSubsystem::getPositionIncrement(){
//     return 2*PI*revPerBall;
// }
    

} //namespace subsystems 