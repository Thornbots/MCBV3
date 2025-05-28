#include "DoubleIndexerSubsystem.hpp"

namespace subsystems
{

DoubleIndexerSubsystem::DoubleIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index1, tap::motor::DjiMotor* index2)
    : IndexerSubsystem(drivers, index1, ShotCounter::BarrelType::TURRET_17MM_1), // Call base class constructor
    other(drivers, index2, ShotCounter::BarrelType::TURRET_17MM_2)
{}

void DoubleIndexerSubsystem::initialize() {
    IndexerSubsystem::initialize();
    other.initialize();
}

void DoubleIndexerSubsystem::refresh() {
    IndexerSubsystem::refresh();
    other.refresh();
}

float DoubleIndexerSubsystem::indexAtRate(float ballsPerSecond){
    return IndexerSubsystem::indexAtRate(ballsPerSecond/2) + other.indexAtRate(ballsPerSecond/2);
}

void DoubleIndexerSubsystem::indexAtMaxRate(){
    IndexerSubsystem::indexAtMaxRate();
    other.indexAtMaxRate();
}

void DoubleIndexerSubsystem::stopIndex() {
    IndexerSubsystem::stopIndex();
    other.stopIndex();
}

void DoubleIndexerSubsystem::unjam() {
    IndexerSubsystem::unjam();
    other.unjam();
}

float DoubleIndexerSubsystem::getNumBallsShot(){
    return IndexerSubsystem::getNumBallsShot() + other.getNumBallsShot();
}

void DoubleIndexerSubsystem::resetBallsCounter() {
    IndexerSubsystem::resetBallsCounter();
    other.resetBallsCounter();
}

float DoubleIndexerSubsystem::getBallsPerSecond(){
    return IndexerSubsystem::getBallsPerSecond() + other.getBallsPerSecond();
}


} // namespace subsystems
