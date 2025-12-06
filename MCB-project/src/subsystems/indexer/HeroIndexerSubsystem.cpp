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
    IndexerSubsystem::refresh();
    bottom.refresh();
}

float HeroIndexerSubsystem::indexAtRate(float inputBallsPerSecond){
    IndexerSubsystem::counter.enable();
    bottom.counter.enable();

    // bottom may be jammed, so top will react to that
    return IndexerSubsystem::indexAtRate(bottom.indexAtRate(inputBallsPerSecond));
}

float HeroIndexerSubsystem::loadAtRate(float inputBallsPerSecond){
    IndexerSubsystem::counter.disable();
    bottom.counter.disable();

    // bottom may be jammed, so top will react to that
    return IndexerSubsystem::indexAtRate(bottom.indexAtRate(inputBallsPerSecond));
}

bool HeroIndexerSubsystem::isProjectileAtBeam(){
    return !Board::DigitalInPinB12::read(); // Assuming PF0 is the break beam sensor
}

float HeroIndexerSubsystem::getActualBallsPerSecond() {
    return std::min(IndexerSubsystem::getActualBallsPerSecond(), bottom.getActualBallsPerSecond());
}


} // namespace subsystems