#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/indexer/IndexerSubsystem.hpp"

#include "drivers.hpp"

namespace commands
{
using subsystems::IndexerSubsystem;
using tap::communication::serial::Remote;

class IndexerIdleCommand : public tap::control::Command
{
public:
    IndexerIdleCommand(src::Drivers* drivers, IndexerSubsystem* indexer)
        : drivers(drivers),
          indexer(indexer)
    {
        addSubsystemRequirement(indexer);
    }

    void initialize() override {};

    void execute() override {    
        indexer->idle();
    }

    void end(bool) override {};

    bool isFinished() const override {return false;}

    const char* getName() const override { return "indexer idle command"; }

private:
    src::Drivers* drivers;
    IndexerSubsystem* indexer;
};
}  // namespace commands