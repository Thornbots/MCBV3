#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/indexer/IndexerSubsystem.hpp"
#include "subsystems/cv/ComputerVisionSubsystem.hpp"

#include "drivers.hpp"

namespace commands
{
using subsystems::GimbalSubsystem;
using subsystems::IndexerSubsystem;
using subsystems::ComputerVisionSubsystem;
using tap::communication::serial::Remote;

class AutoAimCommand : public tap::control::Command
{
public:
    AutoAimCommand(src::Drivers* drivers, GimbalSubsystem* gimbal, IndexerSubsystem* indexer, ComputerVisionSubsystem* cv)
        : drivers(drivers),
          gimbal(gimbal),
          indexer(indexer),
          cv(cv)
    {
        addSubsystemRequirement(gimbal);
        addSubsystemRequirement(indexer);
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "autoaim command"; }
    


private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;
    IndexerSubsystem* indexer;
    ComputerVisionSubsystem* cv;

    bool isCalibrated = false;

    float yaw = 0.0f, pitch = 0.0f;

};
}  // namespace commands