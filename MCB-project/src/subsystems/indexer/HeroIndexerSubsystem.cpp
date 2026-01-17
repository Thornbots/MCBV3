#include "HeroIndexerSubsystem.hpp"

namespace subsystems
{

HeroIndexerSubsystem::HeroIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* indexTop, tap::motor::DjiMotor* indexBottom)
    : IndexerSubsystem(drivers, indexTop), //needs one motor to know disconnect, could be either
    unitTop(   drivers, indexTop,    REV_PER_BALL/GEAR_RATIO,        REV_PER_BALL*GEAR_RATIO),
    unitBottom(drivers, indexBottom, REV_PER_BALL_BOTTOM/GEAR_RATIO, REV_PER_BALL_BOTTOM*GEAR_RATIO),
    counter(drivers, ShotCounter::BarrelType::TURRET_42MM, indexTop)
{}

void HeroIndexerSubsystem::finishInitialize() {
    unitTop.initialize();
    unitBottom.initialize();
    Board::DigitalInPinB12::configure(modm::platform::Gpio::InputType::Floating); //initialze beambreak
    
    state = HeroIndexerState::LOADING_THEN_DONE;
}

void HeroIndexerSubsystem::finishRefresh() {
    // drivers->leds.set(tap::gpio::Leds::Green, isProjectileAtBeam());
    
    // state transitions
    if(state==HeroIndexerState::DONE && !isProjectileAtBeam()){
        // we thought we were done, we shouldn't be
        // maybe we manual unjammed
        state = HeroIndexerState::LOADING_THEN_DONE;
    }
    if(state == HeroIndexerState::INDEXING && !isProjectileAtBeam()){
        // ball has left where we can see it
        state = HeroIndexerState::INDEXING_EXTRA; //but not actually left the barrel yet
        counter.resetRecentBallsByEncoderCounter();
    }
    if(state == HeroIndexerState::INDEXING_EXTRA && counter.getRecentNumBallsShotByEncoder()>=INDEXING_EXTRA_BALLS){
        // done shooting, we need to load
        state = HeroIndexerState::LOADING_THEN_DONE;
    }
    if(state == HeroIndexerState::LOADING_THEN_DONE && isProjectileAtBeam()){
        // done loading
        state = HeroIndexerState::DONE;
    }
    
    
    // motor movements
    if(!drivers->remote.isConnected() || state==HeroIndexerState::DONE || isStopped){
        // stop both
        unitTop.oldVelocityControl(0);
        unitBottom.oldVelocityControl(0);
    } else if(isManualUnjamming) {
        //the user might want to remove the ball at the beambreak
        state = HeroIndexerState::LOADING_THEN_DONE;
        unitTop.oldVelocityControl(UNJAM_BALL_PER_SECOND); 
        unitBottom.oldVelocityControl(UNJAM_BALL_PER_SECOND);
    } else if(state==HeroIndexerState::INDEXING || state==HeroIndexerState::INDEXING_EXTRA) {
        // what speed should this happen at? Old system would set this at 20Hz in HeroControl. Maybe make this a constant.
        unitTop.oldVelocityControl(20);
        unitBottom.oldVelocityControl(20);
    } else if(state==HeroIndexerState::LOADING_THEN_DONE){
        unitTop.oldVelocityControl(LOAD_BALL_PER_SECOND); //top needs to spin too when loading
        unitBottom.oldVelocityControl(LOAD_BALL_PER_SECOND);
    }

    counter.update();
}

void HeroIndexerSubsystem::finishStopIndex() {
    // state = HeroIndexerState::STOPPED;
}


bool HeroIndexerSubsystem::isProjectileAtBeam(){
    return !Board::DigitalInPinB12::read(); // Assuming PF0 is the break beam sensor
}

bool HeroIndexerSubsystem::tryShootOnce() { 
    bool r = canShoot();
    if(r) forceShootOnce();
    return r;
}

void HeroIndexerSubsystem::forceShootOnce() {
    if(state==HeroIndexerState::DONE) {
        if(isProjectileAtBeam()) {
            counter.resetRecentBallsByEncoderCounter();
            state = HeroIndexerState::INDEXING;
            justShot();
        } else {
            state = HeroIndexerState::LOADING_THEN_DONE;
        }
    }
}


const char* HeroIndexerSubsystem::getStateString(){
    switch (state)
    {
    case HeroIndexerState::INDEXING:
        return "indexing";
    case HeroIndexerState::LOADING_THEN_DONE:
        return "loading";
    case HeroIndexerState::INDEXING_EXTRA:
        return "indexing extra";
    case HeroIndexerState::DONE:
        return "done";
    default:
        return "default";
    }
}



float HeroIndexerSubsystem::getEstHeatRatio(){
    return counter.getEstHeatRatio();
}
bool HeroIndexerSubsystem::heatAllowsShooting(){
    return counter.canShootAgain();
}

float HeroIndexerSubsystem::getTotalNumBallsShot(){
    // return counter.getTotalNumBallsShotByEncoder(); //would include loading movement
    return counter.getTimesIncremented();
}

} // namespace subsystems