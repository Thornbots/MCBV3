#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/indexer/IndexerSubsystem.hpp"
#include "subsystems/jetson/JetsonSubsystem.hpp"

#include "drivers.hpp"

//maybe soon do inheritance, for final assessment this is what AutoAimCommand was previously

namespace commands
{
using subsystems::GimbalSubsystem;
using subsystems::IndexerSubsystem;
using subsystems::JetsonSubsystem;
using tap::communication::serial::Remote;

class AutoAimAndFireCommand : public tap::control::Command
{
public:
    AutoAimAndFireCommand(src::Drivers* drivers, GimbalSubsystem* gimbal, IndexerSubsystem* indexer, JetsonSubsystem* cv)
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
    JetsonSubsystem* cv;

    bool isCalibrated = false;
    bool isShooting = false;
    int shoot = 0;

    float yaw = 0.0f, pitch = 0.0f;
    float yawvel = 0.0f, pitchvel = 0.0f;
    uint32_t lastSeenTime = 0;
};
}  // namespace commands