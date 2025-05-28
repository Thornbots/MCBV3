#include "HeroIndexerSubsystem.hpp"

namespace subsystems
{

HeroIndexerSubsystem::HeroIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* indexTop, tap::motor::DjiMotor* indexBottom)
    : IndexerSubsystem(drivers, indexTop, ShotCounter::BarrelType::TURRET_42MM), // Call base class constructor with the 42 barrel
    bottom(drivers, indexBottom, ShotCounter::BarrelType::TURRET_42MM, REV_PER_BALL_BOTTOM)
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

float HeroIndexerSubsystem::indexAtRate(float ballsPerSecond){
    IndexerSubsystem::indexAtRate(ballsPerSecond);
    return bottom.indexAtRate(ballsPerSecond);
}

float HeroIndexerSubsystem::loadAtRate(float ballsPerSecond){
    return bottom.indexAtRate(ballsPerSecond);
}

void HeroIndexerSubsystem::indexAtMaxRate(){
    IndexerSubsystem::indexAtMaxRate();
    bottom.indexAtMaxRate();
}

bool HeroIndexerSubsystem::isProjectileAtBeam(){
    return !Board::DigitalInPinB12::read(); // Assuming PF0 is the break beam sensor
}


} // namespace subsystems