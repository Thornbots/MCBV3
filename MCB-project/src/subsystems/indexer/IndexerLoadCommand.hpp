#pragma once

#include "tap/control/command.hpp"

#include "subsystems/indexer/HeroIndexerSubsystem.hpp"

#include "drivers.hpp"

namespace commands
{
using subsystems::HeroIndexerSubsystem;

class IndexerLoadCommand : public tap::control::Command
{
public:

IndexerLoadCommand(src::Drivers* drivers, HeroIndexerSubsystem* indexer)
        : drivers(drivers),
          indexer(indexer)
    {
        addSubsystemRequirement(indexer);
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "indexer load command"; }

private:
    src::Drivers* drivers;
    HeroIndexerSubsystem* indexer;
};
}  // namespace commands