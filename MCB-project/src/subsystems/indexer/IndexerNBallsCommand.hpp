#pragma once

#include "tap/control/command.hpp"

#include "subsystems/indexer/IndexerSubsystem.hpp"

#include "drivers.hpp"

namespace commands
{
using subsystems::IndexerSubsystem;

class IndexerNBallsCommand : public tap::control::Command
{
public:
    IndexerNBallsCommand(src::Drivers* drivers, IndexerSubsystem* indexer, int numBalls, float ballsPerSecond)
        : drivers(drivers),
          indexer(indexer),
          numBalls(numBalls),
          ballsPerSecond(ballsPerSecond)
    {
        addSubsystemRequirement(indexer);
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "indexer N balls command"; }

private:
    src::Drivers* drivers;
    IndexerSubsystem* indexer;
    int numBalls;
    int numBallsRemaining;
    float ballsPerSecond;

    tap::arch::PeriodicMilliTimer timer;
};
}  // namespace commands