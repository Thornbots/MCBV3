#pragma once
#include "IndexerSubsystem.hpp"

namespace subsystems
{

class DoubleIndexerSubsystem : public IndexerSubsystem
{

public:

    DoubleIndexerSubsystem(src::Drivers* drivers, tap::motor::DjiMotor* index1, tap::motor::DjiMotor* index2);

    ~DoubleIndexerSubsystem() {}

    virtual void initialize();

    virtual void refresh() override;

    virtual float indexAtRate(float ballsPerSecond);
    virtual void indexAtMaxRate();

    virtual void stopIndex();

    virtual void unjam();

    virtual float getNumBallsShot();

    virtual void resetBallsCounter();

    virtual float getBallsPerSecond();

private:
IndexerSubsystem other;

};

} // namespace subsystems
