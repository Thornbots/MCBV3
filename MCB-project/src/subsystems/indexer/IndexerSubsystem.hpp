#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"
#include "tap/control/subsystem.hpp"

#include "ShotCounter.hpp"
#include "drivers.hpp"

namespace subsystems
{

class IndexerSubsystem : public tap::control::Subsystem
{

protected:  // Private Variables
src::Drivers* drivers;
tap::motor::DjiMotor* motorIndexer;
tap::algorithms::SmoothPid indexPIDController;
ShotCounter counter;

float ballsPerSecond = 0.0f;
static constexpr int MAX_INDEX_RPM = 17000;
int32_t indexerVoltage = 0;

public:  // Public Methods

IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index);
IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index, ShotCounter::BarrelType barrel);

~IndexerSubsystem() {}

virtual void initialize();

void refresh() override;

virtual float indexAtRate(float ballsPerSecond);
virtual void indexAtMaxRate();

void setTargetMotorRPM(int targetMotorRPM);

virtual void stopIndex();

void unjam();

float getNumBallsShot();

void resetBallsCounter();

float getBallsPerSecond();

private:  // Private Methods
};
} //namespace subsystems