#pragma once
#include "IndexerSubsystem.hpp"

namespace subsystems
{

class HeroIndexerSubsystem : public IndexerSubsystem
{

public:
    // Additional Motor Constants (if necessary)

    HeroIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* indexTop, tap::motor::DjiMotor* indexBottom);

    ~HeroIndexerSubsystem() {}

    virtual void initialize();

    virtual void refresh() override;

    virtual float indexAtRate(float inputBallsPerSecond);
    float loadAtRate(float inputBallsPerSecond);
    
    virtual void stopIndex();
    
    virtual void unjam(bool isAuto);

    //counter stuff doesn't change

    virtual bool isProjectileAtBeam();

    virtual float getActualBallsPerSecond();
private:

IndexerSubsystem bottom;


enum class HeroIndexerState : uint8_t {
    STOPPED = 0,  //not powering motor at all
    INDEXING = 1,  //running regular indexing behavior until says there isn't a ball, will go back to loading
    LOADING_THEN_DONE = 2,  //loads until beam says there is a ball, then goes to STOPPED
    LOADING_THEN_INDEX = 3,   //loads until beam says there is a ball, then goes to INDEXING
    INDEXING_EXTRA = 4, //after firing, continue moving after the beambreak says the shot has left, defined by INDEXING_EXTRA_BALLS
    DONE = 5,  //holding place, proving power but not trying to move anywhere
};

HeroIndexerState state = HeroIndexerState::STOPPED; //autounjamming doesn't transition state

float startingBalls = 0; //for INDEXING_EXTRA. 
// Maybe in the future make loading a lot like homing, 
// every time you shoot you need to rehome, except it is homing by indexing forwards (same velocity control homing standard and sentry do) 
// Once beam breaks, move an extra amount forward to be right on the edge of firing, 
// then when wanting shoot move a specific amount forward (as quickly as possible because it is position control)
float extraBallsPerSecond = 0;
};

} // namespace subsystems