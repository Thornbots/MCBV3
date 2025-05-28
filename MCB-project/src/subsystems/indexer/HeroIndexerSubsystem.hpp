#ifdef HERO
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

    virtual float indexAtRate(float ballsPerSecond);
    float loadAtRate(float ballsPerSecond);
    virtual void indexAtMaxRate();

    virtual void stopIndex();

    virtual void unjam();

    //counter stuff doesn't change

    bool isProjectileAtBeam();

private:
IndexerSubsystem bottom;

};

} // namespace subsystems
#endif