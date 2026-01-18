#include "IndexerSubsystem.hpp"
#include <cmath>
#include "IndexerSubsystemConstants.hpp"

namespace subsystems {
using namespace tap::communication::serial;

IndexerSubsystem::IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index)
    : tap::control::Subsystem(drivers),
    drivers(drivers),
    motorIndexer(index)
    {}

void IndexerSubsystem::initialize() {
    drivers->commandScheduler.registerSubsystem(this);
    timer.stop();
    
    finishInitialize(); //initialize motors and any other things Single and Hero want to do
}

void IndexerSubsystem::refresh() {
    
    // check if indexAtRate or indexNShotsAtRate should call tryShootOnce
    if(timer.execute()) {
        tryShootOnce();                                //shoot
        if(numShotsRemaining>0) numShotsRemaining--;   //count it if we should (if -1 then we don't count it)
        if(numShotsRemaining==0) stopIndexingAtRate(); //if it is now zero, stop
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
    numShotsRemaining = numShots;
    if(shotImmediately) 
        numShotsRemaining--; //if shot one, count it
            
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


bool IndexerSubsystem::canShoot() {
    // return true;
    return isIndexOnline() && refPoweringIndex() && heatAllowsShooting() && isProjectileAtBeam() && tap::arch::clock::getTimeMilliseconds()-lastShotTime>=MIN_SHOT_FREQ;
}

void IndexerSubsystem::justShot() {
    lastShotTime = tap::arch::clock::getTimeMilliseconds();
}
    
int IndexerSubsystem::getMeasurement() {
    return motorIndexer->getTorque();
}

bool IndexerSubsystem::autoUnjamTrigger(tap::motor::DjiMotor* unjamMotor) {
    return unjamMotor->getTorque()>AUTO_UNJAM_TORQUE_THRES;
}

void IndexerSubsystem::doAutoUnjam(tap::motor::DjiMotor* unjamMotor, tap::arch::MilliTimeout& timeoutUnjam, bool& isAutoUnjamming) {
    //if unjamming
    if(isAutoUnjamming){
        if(timeoutUnjam.isExpired()){
            timeoutUnjam.stop();
            isAutoUnjamming = false;
        }
    }

    //if we are here, isAutoUnjamming is false or inputBallsPerSecond<=0
    //if we are slow and trying to go forward
    if(autoUnjamTrigger(unjamMotor)){
        if(timeoutUnjam.isStopped()){
            timeoutUnjam.restart(AUTO_UNJAM_TIME_UNDER_THRESH*1000);
        }

        if(timeoutUnjam.isExpired()){
            isAutoUnjamming = true;
            timeoutUnjam.restart(AUTO_UNJAM_TIME_UNJAMMING*1000);
            IndexerSubsystem::indexAtRate(UNJAM_BALL_PER_SECOND);
        }
    }
}

} //namespace subsystems 