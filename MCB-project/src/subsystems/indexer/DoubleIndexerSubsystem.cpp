#include "DoubleIndexerSubsystem.hpp"

namespace subsystems
{

DoubleIndexerSubsystem::DoubleIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index1, tap::motor::DjiMotor* index2, bool doHoming)
    : IndexerSubsystem(drivers, index1, doHoming, ShotCounter::BarrelType::TURRET_17MM_1), // Call base class constructor
    other(drivers, index2, doHoming, ShotCounter::BarrelType::TURRET_17MM_2)
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
    float motor1NumBallsShot = IndexerSubsystem::getTotalNumBallsShot();
    float motor2NumBallsShot = other.getTotalNumBallsShot();
    // error represents the shortest path to the desired phase shift
    float error = fmod(motor1NumBallsShot - motor2NumBallsShot, 1.0) - 0.5;
    // set correctionFactor to a multiple of error, or 0 if ballsPerSecond <= 0
    float correctionFactor = error * abs(ballsPerSecond/2) * 0.25;

    return IndexerSubsystem::indexAtRate(ballsPerSecond/2 - correctionFactor) + other.indexAtRate(ballsPerSecond/2 + correctionFactor);

    // return IndexerSubsystem::indexAtRate(ballsPerSecond/2) + other.indexAtRate(ballsPerSecond/2);
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
