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
    
    if(drivers->remote.isConnected()) {
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
    
    counter.update();
}

void SingleIndexerSubsystem::finishStopIndex() {
    temporaryVelocityControl = false;
    shouldIndexNearest = true;
}

void SingleIndexerSubsystem::forceShootOnce() {
    unit.shootOnce();
    counter.incrementTargetNumBalls();
    justShot();
}

bool SingleIndexerSubsystem::tryShootOnce() {
    bool r = canShoot();
    if(r) forceShootOnce();
    return r;
}

float SingleIndexerSubsystem::getEstHeatRatio(){
    return counter.getEstHeatRatio();
}
bool SingleIndexerSubsystem::heatAllowsShooting(){
    return counter.canShootAgain();
}

float SingleIndexerSubsystem::getTotalNumBallsShot(){
    return counter.getTimesIncremented();
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
        unit.doAfterHomingOffset();
    }
    if(timeoutHome.isExpired()){
        done = true;
        homingState = HomingState::GAVE_UP_HOMING;
        unit.doGiveUpHomingOffset();
    }
    if(done){
        homingCounter = 0; //reset for next time
        temporaryVelocityControl = false;
        unit.velocityControl(0);
    }
}

void SingleIndexerSubsystem::indexNearest() {
    unit.indexNearest();
}


} //namespace subsystems 