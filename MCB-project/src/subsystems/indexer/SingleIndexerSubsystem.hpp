#pragma once
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/control/subsystem.hpp"

#include "ShotCounter.hpp"
#include "IndexerUnit.hpp"
#include "IndexerSubsystem.hpp"
#include "drivers.hpp"

namespace subsystems
{

class SingleIndexerSubsystem : public IndexerSubsystem
{



public:  // Public Methods

    SingleIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index);

    ~SingleIndexerSubsystem() {}

    virtual void finishInitialize();

    virtual void finishRefresh();

    virtual void finishStopIndex();

    // be careful of looping here. 
    // void unjam() {unjam(false);};
    // virtual void unjam(bool isAuto);

    // virtual float getNumBallsShot();
    // virtual float getTotalNumBallsShot();
    // virtual void resetBallsCounter();

    virtual bool tryShootOnce();
    virtual void forceShootOnce();

    virtual float getEstHeatRatio();
    virtual bool heatAllowsShooting();
    virtual float getTotalNumBallsShot();


    void indexNearest(); //indexes to nearest shot position after finishing a position move

protected:

    virtual void doHomingTransitions(); //used to transition into the homing state when needed
    virtual void homeIndexer(); //called when in the homing state


// bool doAutoUnjam(float inputBallsPerSecond);


private:
    ShotCounter counter; //was public

    IndexerUnit unit;

    tap::arch::MilliTimeout timeoutUnjam;
    bool isAutoUnjamming = false;
    bool shouldIndexNearest = false;

    tap::arch::MilliTimeout timeoutHome;
    enum class HomingState : uint8_t {
        NEED_TO_HOME = 1,    // waiting until motor turns on (or imu to be done) to start homing
        HOMING = 2,          // the timer is going, the motor is spinning
        HOMED = 3,           // homed sucessfully, by motor being stalled
        GAVE_UP_HOMING = 4   // timer ran out, we are out of physical shots (probably)
    };

    int homingCounter = 0;

    // hero doesn't want position control
    bool doPositionControl;

    // for something like unjamming on standard
    bool temporaryVelocityControl = false;

    HomingState homingState;


};
} //namespace subsystems