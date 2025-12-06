#pragma once
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"
#include "tap/control/subsystem.hpp"

#include "ShotCounter.hpp"
#include "IndexerController.hpp"
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

// revolutions of OUTPUT SHAFT per ball. Different than the ones per robot in IndexerSubsystemConstants, because those are motor shaft (before the gear box)
float revPerBall;

float ballsPerSecond = 0.0f;
static constexpr int MAX_INDEX_RPM = 17000;


tap::arch::MilliTimeout timeoutUnjam;
bool isAutoUnjamming = false;

float targetIndexerPosition = 0;
    
IndexerController indexerController;


tap::arch::MilliTimeout timeoutHome;
enum class HomingState : uint8_t {
    DONT_HOME = 0,       // hero doesn't home, it has a beambreak. Standard and sentry do it for position control
    NEED_TO_HOME = 1,    // waiting until motor turns on (or imu to be done) to start homing
    HOMING = 2,          // the timer is going, the motor is spinning
    HOMED = 3,           // homed sucessfully, by motor being stalled
    GAVE_UP_HOMING = 4   // timer ran out, we are out of physical shots (probably)
};

private:
int homingCounter = 0;

// hero doesn't want position control
bool doPositionControl;

// for something like unjamming on standard
bool temporaryVelocityControl = false;

HomingState homingState;

int getIndexerVoltage(float currentPosition, float currentVelocity, float targetPosition, float inputVelocity, float deltaT);

public:  // Public Methods

IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index, bool doHoming, bool doPositionControl);
IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index, bool doHoming, bool doPositionControl, ShotCounter::BarrelType barrel);
IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index, bool doHoming, bool doPositionControl, ShotCounter::BarrelType barrel, float revPerBall);

~IndexerSubsystem() {}

virtual void initialize();

virtual void refresh() override;

virtual float indexAtRate(float ballsPerSecond);

virtual void stopIndex();

// be careful of looping here. 
void unjam() {unjam(false);};
virtual void unjam(bool isAuto);

virtual float getNumBallsShot();
virtual float getTotalNumBallsShot();

virtual void resetBallsCounter();

// doesn't check heat. use heatAllowsShooting() to know if you should call this
virtual void incrementTargetNumBalls();

virtual float getBallsPerSecond();
virtual void setBallsPerSecond(float newBallsPerSecond);

virtual float getActualBallsPerSecond();

// non hero always returns true
virtual bool isProjectileAtBeam();

// if the ref system thinks it's powering the motor. Either there is a delay until power is given or the motor takes time to turn on.
virtual bool refPoweringIndex();

// if the motor thinks it's online. There is a delay after the ref system thinks it is giving power
virtual bool isIndexOnline();

virtual int32_t getEstHeat();
virtual bool heatAllowsShooting();
virtual bool canShoot();


void homeIndexer();

void indexNearest(); //indexes to nearest shot position after finishing a position move



protected:

bool doAutoUnjam(float inputBallsPerSecond);

// used for getIndexerVoltage
float getCurrentOutputVelo();

// uses targetIndexerPosition to do position control
void positionControl();

// uses ballsPerSecond to do velocity control
void velocityControl();

// position increment of output shaft
float getPositionIncrement();

};
} //namespace subsystems