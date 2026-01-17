#pragma once
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/control/subsystem.hpp"

// #include "IndexerController.hpp"
#include "drivers.hpp"

namespace subsystems
{

    
// a superclass for SingleIndexerSubsystem and HeroIndexerSubsystem
// allows commands to say to shoot at a rate, shoot once, unjam...
// and single and hero can handle that differently
// it used to be that SingleIndexerSubsystem's responibilities were in IndexerSubsystem, 
// and HeroIndexerSubsystem would override things. Hero broke when we changed something (position control)
// that would have only applied to SingleIndexerSubsystem if it had existed.
class IndexerSubsystem : public tap::control::Subsystem
{
    
private:
uint32_t lastShotTime = 0; //ms, for the 20Hz cooldown
static constexpr uint32_t MIN_SHOT_FREQ = 50; //ms, for the 20Hz cooldown

int numShotsRemaining = 0; //for indexNShotsAtRate. Decrements each time tryShootOnce gets called. Is -1 for indexAtRate.
float shotsPerSecond = 0; //is 0 if not continuously firing, is positive if indexAtRate or indexNShotsAtRate is working.
tap::arch::PeriodicMilliTimer timer;


protected:  
src::Drivers* drivers;
tap::motor::DjiMotor* motorIndexer; //for checking if it is online

// single and hero should check this
bool isManualUnjamming = false;

// single and hero could check this
bool isStopped = true;

// revolutions of OUTPUT SHAFT per ball. Different than the ones per robot in IndexerSubsystemConstants, because those are motor shaft (before the gear box)
// float revPerBall;
// float targetIndexerPosition = 0;
// IndexerController indexerController;


// int getIndexerVoltage(float currentPosition, float currentVelocity, float targetPosition, float inputVelocity, float deltaT);



// setting a virtual method equal to 0 is like an abstract method in java: it isn't implemented in this class, but it is in a subclass.

// IndexerSubsystem does things in initialize, Single and Hero override this to initialize things.
virtual void finishInitialize() = 0;
// IndexerSubsystem does things in refresh, Single and Hero override this to update.
virtual void finishRefresh() = 0;

public:

IndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index);

~IndexerSubsystem() {}

// only registers the subsystem here
virtual void initialize();

// sets isHoming using shouldStartHoming and shouldEndHoming, and handles indexAtRate and indexNShotsAtRate calling tryShootOnce.
virtual void refresh();

// Sets the index to shoot at a fixed rate until stopped.
// This can be stopped by calling tryShootOnce() or forceShootOnce(), or indexAtRate(0), or stopIndex().
// Avoids overheating.
// If inputShotsPerSecond is negative or 0, then this behaves like stopIndexingAtRate().
// If inputShotsPerSecond is above 20, then this behaves like it was 20 because of canShoot()'s 20Hz cooldown between shots.
// Returns true if it shot immediately, false if it didn't (because the index was already indexing at a rate)
bool indexAtRate(float inputShotsPerSecond);

// Sets the index to shoot at a fixed rate until tryShootOnce() has benn called the target number of times, or until stopped.
// see indexAtRate() for more info.
// If numShots is negative or 0, then this behaves like stopIndexingAtRate().
// Returns true if it shot immediately, false if it didn't (because the index was already indexing at a rate)
bool indexNShotsAtRate(float inputShotsPerSecond, int numShots);

// stops indexAtRate or indexNShotsAtRate from continuing.
void stopIndexingAtRate();

// if indexNShotsAtRate shot all it's shots, or indexAtRate or indexNShotsAtRate were stopped some other way.
bool isDoneIndexingAtRate();

// Stops the motors from moving as quick as possible, avoiding shooting any more shots.
// Stops indexAtRate and indexNShotsAtRate from shooting more shots.
void stopIndex();


virtual void finishStopIndex() = 0; 


// stops indexAtRate and unjam, but allows the indexer to still move
void idle();

// Sets the index to spin backwards until stopped.
// Stopped in the same way as indexAtRate().
void manualUnjam();



// uses canShoot() to determine if shooting is possible. 
// If it can shoot, shoots once, and returns true.
// Otherwise this is ignored and returns false.
virtual bool tryShootOnce() = 0;

// doesn't check heat. use heatAllowsShooting() to know if you should call this
// Might be useful if you intentionally want to overheat.
virtual void forceShootOnce() = 0; 
// on hero, should force ignore beambreak?
// It depends on what the robot pilot wants 'force shoot' to do.
// Would have been good in the past, killing yourself to kill the base.


//indexes to nearest shot position after finishing a position move. Would only apply to Single for now. And Single would take care of this automatically.
// void indexNearest(); 


// If the beambreak sensor senses that there is a shot ready.
// non heros always returns true, as far as they know there is always a shot ready.
virtual bool isProjectileAtBeam();

// if the ref system thinks it's powering the motor. Either there is a delay until power is given or the motor just takes a lot of time to turn on.
bool refPoweringIndex();

// if the motor thinks it's online. There is a delay after the ref system thinks it is returning power, and there is a delay after the ref system thinks it took power.
bool isIndexOnline();

// gives a number (ideally) between 0 and 1. 0 is no heat, 1 is full of heat. If forceShootOnce() is used then it could be above 1.
virtual float getEstHeatRatio() = 0;

// used by ui (PredictedRemainingShotsIndicator)
virtual float getTotalNumBallsShot() = 0;

// If heat (kept track on the robot, not through the ref system) allows shooting another shot.
virtual bool heatAllowsShooting() = 0;

// If isIndexOnline(), refPoweringIndex(), heatAllowsShooting(), and isProjectileAtBeam() all return true.
// Also has a 20Hz cooldown to prevent tryShootOnce() called too quickly from shooting too quickly.
virtual bool canShoot();



protected:

// // if homing (or loading for hero) should start. Check isHoming for if the robot is currently homing.
// virtual bool shouldStartHoming() = 0;
// // if homing (or loading for hero) should stop. Check isHoming for if the robot is currently homing.
// virtual bool shouldEndHoming() = 0;
// bool isHoming; //set by IndexerSubsystem, used by Single and Hero in their finishRefresh()

// Single and Hero would call this when they shoot.
// This is for the 20Hz cooldown in canShoot().
void justShot();

};
} //namespace subsystems