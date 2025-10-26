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


public:
ShotCounter counter;

protected:  // Private Variables
src::Drivers* drivers;
tap::motor::DjiMotor* motorIndexer;
tap::algorithms::SmoothPid indexPIDController;
float revPerBall;

float ballsPerSecond = 0.0f;
float initialHomingNumBalls = 0.0f;
static constexpr int MAX_INDEX_RPM = 17000;
int32_t indexerVoltage = 0;


tap::arch::MilliTimeout timeoutUnjam;
bool isAutoUnjamming = false;


tap::arch::MilliTimeout timeoutHome;
enum class HomingState : uint8_t {
    DONT_HOME = 0,           // hero doesn't home, it has a beambreak. Standard might home. Sentry needs it
    NEED_TO_HOME = 1,        // waiting until motor turns on (or imu to be done) to start homing
    HOMING = 2,              // the timer is going, the motor is spinning
    HOMED = 3,               // homed sucessfully, by motor being stalled and we are moved to where we need to be
    GAVE_UP_HOMING = 4,      // timer ran out, we are out of physical shots (probably)
    WAITING_AFTER_HOMING = 5,// wait for a projectile to fall in the slot right before the flywheels before moving forward to put it really close to flywheels
    MOVING_AFTER_HOMING = 6  // homed sucessfully, by motor being stalled and we still need to move where we need to be
};

private:
int homeTimeoutCounter = 0;

HomingState homingState;

public:  // Public Methods

IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index, bool doHoming);
IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index, bool doHoming, ShotCounter::BarrelType barrel, float initialHomingNumBalls);
IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index, bool doHoming, ShotCounter::BarrelType barrel, float initialHomingNumBalls, float revPerBall);

~IndexerSubsystem() {}

virtual void initialize();

virtual void refresh() override;

virtual float indexAtRate(float ballsPerSecond);
virtual void indexAtMaxRate();

void stopIndex();

void unjam();

virtual float getNumBallsShot();
virtual float getTotalNumBallsShot();

virtual void resetBallsCounter();

virtual float getBallsPerSecond();

virtual float getActualBallsPerSecond();

// non hero always returns true
virtual bool isProjectileAtBeam();

virtual bool isIndexOnline();


void homeIndexer();

private:  // Private Methods

void setTargetMotorRPM(int targetMotorRPM);

protected:

bool doAutoUnjam(float inputBallsPerSecond);


};
} //namespace subsystems