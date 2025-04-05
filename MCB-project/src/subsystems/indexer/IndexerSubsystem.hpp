#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"
#include "tap/control/subsystem.hpp"


#include "drivers.hpp"

namespace subsystems
{

class IndexerSubsystem : public tap::control::Subsystem
{

protected:  // Private Variables
src::Drivers* drivers;
// #if defined(sentry)
tap::motor::DjiMotor* motorIndexer;//{drivers, tap::motor::MotorId::MOTOR4, tap::can::CanBus::CAN_BUS2, false, "Indexer", 0, 0};
// #else
// tap::motor::DjiMotor motorIndexer{drivers, tap::motor::MotorId::MOTOR7, tap::can::CanBus::CAN_BUS2, false, "Indexer", 0, 0};
// #endif
tap::algorithms::SmoothPid indexPIDController;

float ballsPerSecond = 0.0f;
static constexpr float HEAT_PER_BALL = 10.0f;
static constexpr float LATENCY = 0.6f; //expected ref system latency for barrel heat limiting
int32_t indexerVoltage = 0;
int64_t numTicksAtInit = 0;

public:  // Public Methods

IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index);

~IndexerSubsystem() {}

void initialize();

void refresh() override;

virtual void indexAtRate(float ballsPerSecond);

virtual void setTargetMotorRPM(int targetMotorRPM);

virtual void stopIndex();

virtual void unjam();

float getNumBallsShot();

void resetBallsCounter();

float getBallsPerSecond();

private:  // Private Methods
};
} //namespace subsystems