#include "HeroIndexerSubsystem.hpp"

namespace subsystems
{

HeroIndexerSubsystem::HeroIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* indexTop, tap::motor::DjiMotor* indexBottom)
    : IndexerSubsystem(drivers, indexTop, false, ShotCounter::BarrelType::TURRET_42MM), // Call base class constructor with the 42 barrel
    bottom(drivers, indexBottom, false, ShotCounter::BarrelType::TURRET_42MM, REV_PER_BALL_BOTTOM)
{}

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

float HeroIndexerSubsystem::indexAtRate(float inputBallsPerSecond){
    IndexerSubsystem::counter.enable();
    bottom.counter.enable();

    // float top = IndexerSubsystem::indexAtRate(inputBallsPerSecond);
    // return isAutoUnjamming ? top : bottom.indexAtRate(inputBallsPerSecond); //if top decides to unjam both, let it do so, else let bottom decide

    IndexerSubsystem::indexAtRate(inputBallsPerSecond);
    return bottom.indexAtRate(inputBallsPerSecond);
}

float HeroIndexerSubsystem::loadAtRate(float inputBallsPerSecond){
    IndexerSubsystem::counter.disable();
    bottom.counter.disable();

    // float top = IndexerSubsystem::indexAtRate(inputBallsPerSecond);
    // return isAutoUnjamming ? top : bottom.indexAtRate(inputBallsPerSecond);

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

float HeroIndexerSubsystem::getActualBallsPerSecond() {
    return std::min(IndexerSubsystem::getActualBallsPerSecond(), bottom.getActualBallsPerSecond());
}


} // namespace subsystems