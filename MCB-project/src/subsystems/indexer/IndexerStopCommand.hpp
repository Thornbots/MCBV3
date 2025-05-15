#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/indexer/IndexerSubsystem.hpp"

#include "drivers.hpp"

namespace commands
{
using subsystems::IndexerSubsystem;
using tap::communication::serial::Remote;

class IndexerStopCommand : public tap::control::Command
{
public:
    IndexerStopCommand(src::Drivers* drivers, IndexerSubsystem* indexer)
        : drivers(drivers),
          indexer(indexer)
    {
        addSubsystemRequirement(indexer);
    }

    void initialize() override {};

    void execute() override {    indexer->stopIndex();}

    void end(bool) override {};

    bool isFinished() const override {return false;}

    const char* getName() const override { return "indexer stop command"; }

private:
    src::Drivers* drivers;
    IndexerSubsystem* indexer;
};
}  // namespace commands