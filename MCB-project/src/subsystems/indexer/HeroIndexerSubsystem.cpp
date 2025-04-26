#include "HeroIndexerSubsystem.hpp"
#include "IndexerSubsystemConstants.hpp"
// #include "tap/board/board.hpp"

namespace subsystems
{

HeroIndexerSubsystem::HeroIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index1, tap::motor::DjiMotor* index2)
    : IndexerSubsystem(drivers, index1), // Call base class constructor
    bottomIndexer(index2),
    indexPIDController2(PID_CONF_INDEX)
{

    // Any additional initialization for the second motor, if necessary
}

void HeroIndexerSubsystem::initialize() {
    IndexerSubsystem::initialize();
    // Initialize both motors
    bottomIndexer->initialize();     // Initialize the second motor
    Board::BeamBreak::configure(modm::platform::Gpio::InputType::Floating);
}

void HeroIndexerSubsystem::refresh() {
    IndexerSubsystem::refresh();
    // Set the desired output for both motors
    bottomIndexer->setDesiredOutput(indexerVoltage2);   // Second motor (same voltage)
}

void HeroIndexerSubsystem::indexAtRate(float ballsPerSecond){
    IndexerSubsystem::indexAtRate(ballsPerSecond);

    // Check if the firing rate should be limited to prevent overheating
    tap::communication::serial::RefSerial::Rx::TurretData turretData = drivers->refSerial.getRobotData().turret;
    if (drivers->refSerial.getRefSerialReceivingData() && (HEAT_PER_BALL * ballsPerSecond - turretData.coolingRate) * LATENCY > (turretData.heatLimit - turretData.heat17ID2)) {
        ballsPerSecond = turretData.coolingRate / HEAT_PER_BALL;
    }

    setTargetMotor2RPM(ballsPerSecond * 60.0f * REV_PER_BALL_BOTTOM); // compiler being stupid
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
    return Board::BeamBreak::read(); // Assuming PF0 is the break beam sensor
}


} // namespace subsystems
