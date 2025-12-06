#include "HeroIndexerSubsystem.hpp"

namespace subsystems
{

HeroIndexerSubsystem::HeroIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* indexTop, tap::motor::DjiMotor* indexBottom)
    : IndexerSubsystem(drivers, indexTop, false, false, ShotCounter::BarrelType::TURRET_42MM), // Call base class constructor with the 42 barrel
    bottom(drivers, indexBottom, false, false, ShotCounter::BarrelType::TURRET_42MM, REV_PER_BALL_BOTTOM)
{}

void HeroIndexerSubsystem::initialize() {
    // Initialize both motors
    IndexerSubsystem::initialize();
    bottom.initialize();

    Board::DigitalInPinB12::configure(modm::platform::Gpio::InputType::Floating); //initialze beambreak
}

void HeroIndexerSubsystem::refresh() {
    drivers->leds.set(tap::gpio::Leds::Green, isProjectileAtBeam());
    
    IndexerSubsystem::refresh();
    bottom.refresh();
}

// sometimes we'll ignore what the command wants
// indexNBalls is calling this, but if a ball isn't at the beam load instead
float HeroIndexerSubsystem::indexAtRate(float inputBallsPerSecond){
    if(isProjectileAtBeam() || state >= HeroIndexerState::INDEXING_EXTRA){ //indexing extra or done
        // if being told to go zero, that is being done
        if(inputBallsPerSecond==0) state = HeroIndexerState::DONE;
        
        // done indexing extra
        if(state==HeroIndexerState::INDEXING_EXTRA && counter.getTotalNumBallsShot() >= startingBalls+INDEXING_EXTRA_BALLS){
            state = HeroIndexerState::LOADING_THEN_DONE; //not sure if this indexAtRate is continuous fire or not.
            return loadAtRate(LOAD_BALL_PER_SECOND);
        }
        
        // if were stopped or loading, now indexing
        // don't switch away from INDEXING_EXTRA or DONE
        if(state<HeroIndexerState::INDEXING_EXTRA) state = HeroIndexerState::INDEXING;
        
        IndexerSubsystem::counter.enable();
        bottom.counter.enable();
        
        // bottom may be jammed, so top will react to that
        return IndexerSubsystem::indexAtRate(bottom.indexAtRate(inputBallsPerSecond));
    } else {
        // being told to index when we should load, but do the extra
        state=HeroIndexerState::INDEXING_EXTRA;
        startingBalls = counter.getTotalNumBallsShot();
        extraBallsPerSecond = inputBallsPerSecond;
    }
}

void HeroIndexerSubsystem::stopIndex() {
    state = HeroIndexerState::STOPPED;
    
    IndexerSubsystem::stopIndex();
    bottom.stopIndex();
}

void HeroIndexerSubsystem::unjam(bool isAuto) {
    // user is saying to unjam, that stops movement after we reload what they removed
    if(!isAuto) state = HeroIndexerState::LOADING_THEN_DONE;
    
    IndexerSubsystem::unjam(isAuto);
    bottom.unjam(isAuto);
}

float HeroIndexerSubsystem::loadAtRate(float inputBallsPerSecond){
    if(inputBallsPerSecond==0) indexAtRate(0);
    
    if(state==HeroIndexerState::INDEXING_EXTRA) indexAtRate(extraBallsPerSecond);
    
    if(isProjectileAtBeam()){
        // being told to load when we shouldn't
        if(state==HeroIndexerState::LOADING_THEN_DONE)
            loadAtRate(0);
        // otherwise indexAtRateis calling us
    } else {
        if(state==HeroIndexerState::INDEXING) state = HeroIndexerState::LOADING_THEN_INDEX;
        if(state==HeroIndexerState::STOPPED) state = HeroIndexerState::LOADING_THEN_DONE;
        // if already either loading, don't change state
        
        
        IndexerSubsystem::counter.disable();
        bottom.counter.disable();
        
        // bottom may be jammed, so top will react to that
        IndexerSubsystem::indexAtRate(inputBallsPerSecond);
        bottom.indexAtRate(inputBallsPerSecond);
        // return IndexerSubsystem::indexAtRate(bottom.indexAtRate(inputBallsPerSecond));
    }
    return 0;
}

bool HeroIndexerSubsystem::isProjectileAtBeam(){
    return !Board::DigitalInPinB12::read(); // Assuming PF0 is the break beam sensor
}

float HeroIndexerSubsystem::getActualBallsPerSecond() {
    return std::min(IndexerSubsystem::getActualBallsPerSecond(), bottom.getActualBallsPerSecond());
}


} // namespace subsystems