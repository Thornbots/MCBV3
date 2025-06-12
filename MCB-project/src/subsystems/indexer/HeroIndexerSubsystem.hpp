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
    virtual void indexAtMaxRate();

    //counter stuff doesn't change

    bool isProjectileAtBeam();

    virtual float getActualBallsPerSecond();
private:
IndexerSubsystem bottom;

};

} // namespace subsystems