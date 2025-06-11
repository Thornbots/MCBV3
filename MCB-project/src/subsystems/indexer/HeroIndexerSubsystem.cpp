#include "HeroIndexerSubsystem.hpp"

namespace subsystems
{

HeroIndexerSubsystem::HeroIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* indexTop, tap::motor::DjiMotor* indexBottom)
    : IndexerSubsystem(drivers, indexTop, ShotCounter::BarrelType::TURRET_42MM), // Call base class constructor with the 42 barrel
    bottom(drivers, indexBottom, ShotCounter::BarrelType::TURRET_42MM, REV_PER_BALL_BOTTOM)
{timeout.stop();}

void HeroIndexerSubsystem::initialize() {
    // Initialize both motors
    IndexerSubsystem::initialize();
    bottom.initialize();

    Board::DigitalInPinB12::configure(modm::platform::Gpio::InputType::Floating); //initialze beambreak
}

void HeroIndexerSubsystem::refresh() {
    IndexerSubsystem::refresh();
    bottom.refresh();
}

bool HeroIndexerSubsystem::doAutoUnjam(float inputBallsPerSecond) {
    //if unjamming
    if(isAutoUnjamming){
        if(timeout.isExpired()){
            timeout.stop();
            isAutoUnjamming = false;
        } else if (inputBallsPerSecond > 0) {
            //prevent infinite recursion, unjam calls indexAtRate with a negative number
            unjam();
            return true;
        }
    }
    
    //if we are here, isAutoUnjamming is false or inputBallsPerSecond<=0
    //if we are slow and trying to go forward
    if(inputBallsPerSecond > 0 && bottom.getActualBallsPerSecond()<AUTO_UNJAM_BALLS_PER_SEC_THRESH){
        if(timeout.isStopped()){
            timeout.restart(AUTO_UNJAM_TIME_UNDER_THRESH*1000);
        }

        if(timeout.isExpired()){
            isAutoUnjamming = true;
            timeout.restart(AUTO_UNJAM_TIME_UNJAMMING*1000);
            unjam();
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

float HeroIndexerSubsystem::indexAtRate(float inputBallsPerSecond){
    if(doAutoUnjam(inputBallsPerSecond)) return UNJAM_BALL_PER_SECOND;

    IndexerSubsystem::counter.enable();
    bottom.counter.enable();
    IndexerSubsystem::indexAtRate(inputBallsPerSecond);
    return bottom.indexAtRate(inputBallsPerSecond);
}

float HeroIndexerSubsystem::loadAtRate(float inputBallsPerSecond){
    if(doAutoUnjam(inputBallsPerSecond)) return UNJAM_BALL_PER_SECOND;

    IndexerSubsystem::counter.disable();
    bottom.counter.disable();
    IndexerSubsystem::indexAtRate(inputBallsPerSecond);
    return bottom.indexAtRate(inputBallsPerSecond);
}

void HeroIndexerSubsystem::indexAtMaxRate(){
    IndexerSubsystem::indexAtMaxRate();
    bottom.indexAtMaxRate();
}

bool HeroIndexerSubsystem::isProjectileAtBeam(){
    return !Board::DigitalInPinB12::read(); // Assuming PF0 is the break beam sensor
}


} // namespace subsystems