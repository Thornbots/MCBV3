//ignore this file if HERO is not defined
#ifdef HERO
#include "HeroIndexerSubsystem.hpp"

namespace subsystems
{

HeroIndexerSubsystem::HeroIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* indexTop, tap::motor::DjiMotor* indexBottom)
    : IndexerSubsystem(drivers, indexTop, ShotCounter::BarrelType::TURRET_42MM), // Call base class constructor with the 42 barrel
    bottomIndexer(indexBottom),
    indexPIDController2(PID_CONF_INDEX)
{

    // Any additional initialization for the second motor, if necessary
}

void HeroIndexerSubsystem::initialize() {
    // Initialize both motors
    IndexerSubsystem::initialize();
    bottomIndexer->initialize();     // Initialize the second motor

    Board::DigitalInPinB12::configure(modm::platform::Gpio::InputType::Floating); //initialze beambreak
}

void HeroIndexerSubsystem::refresh() {
    // Set the desired output for both motors
    IndexerSubsystem::refresh();
    bottomIndexer->setDesiredOutput(indexerVoltage2);   // Second motor (same voltage)
}

float HeroIndexerSubsystem::indexAtRate(float ballsPerSecond){
    // IndexerSubsystem will prevent overheat and tell me what ballsPerSecond to use
    ballsPerSecond = IndexerSubsystem::indexAtRate(ballsPerSecond);
    setTargetMotor2RPM(ballsPerSecond * 60.0f * REV_PER_BALL_BOTTOM); // compiler being stupid
    return ballsPerSecond;
}

void HeroIndexerSubsystem::indexAtMaxRate(){
    IndexerSubsystem::indexAtMaxRate();
    setTargetMotor2RPM(MAX_INDEX_RPM);
}

void HeroIndexerSubsystem::setTargetMotor2RPM(int targetMotorRPM){
    indexPIDController2.runControllerDerivateError(targetMotorRPM - bottomIndexer->getShaftRPM(), 1);

    indexerVoltage2 = static_cast<int32_t>(indexPIDController2.getOutput());
}

bool HeroIndexerSubsystem::readBreakBeam() {
    return Board::DigitalInPinB12::read(); // Assuming PF0 is the break beam sensor
}


} // namespace subsystems
#endif