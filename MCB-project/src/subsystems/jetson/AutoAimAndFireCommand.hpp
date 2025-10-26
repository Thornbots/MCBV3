#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/flywheel/FlywheelSubsystem.hpp"
#include "subsystems/flywheel/FlywheelSubsystemConstants.hpp"
#include "subsystems/indexer/IndexerSubsystem.hpp"
#include "subsystems/jetson/JetsonSubsystem.hpp"

#include "AutoDriveCommand.hpp"

#include "drivers.hpp"

//maybe soon do inheritance, for final assessment this is what AutoAimCommand was previously

namespace commands
{
using subsystems::GimbalSubsystem;
using subsystems::IndexerSubsystem;
using subsystems::JetsonSubsystem;
using subsystems::FlywheelSubsystem;
using tap::communication::serial::Remote;

class AutoAimAndFireCommand : public tap::control::Command
{
public:
    AutoAimAndFireCommand(src::Drivers* drivers, GimbalSubsystem* gimbal, IndexerSubsystem* indexer, FlywheelSubsystem* flywheel, JetsonSubsystem* cv, AutoDriveCommand* adc)
        : drivers(drivers),
          gimbal(gimbal),
          indexer(indexer),
          flywheel(flywheel),
          cv(cv),
          adc(adc)
    {
        addSubsystemRequirement(gimbal);
        addSubsystemRequirement(indexer);
        addSubsystemRequirement(flywheel);
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
    FlywheelSubsystem* flywheel;
    JetsonSubsystem* cv;
    AutoDriveCommand* adc;

    bool isCalibrated = false;
    bool isShooting = false;
    int shoot = 0;

    int numCyclesForBurst = 0;
    static constexpr int CYCLES_UNTIL_BURST = 380; //cycles
    static constexpr float BURST_AMOUNT = 0.0; //rad/cycle
    static constexpr float PATROL_SPEED = 0.004; //rad/cycle
    static constexpr int PERSISTANCE = 200;

    float yaw = 0.0f, pitch = 0.0f;
    float yawvel = 0.0f, pitchvel = 0.0f;
    //int flip = 1;
    uint32_t lastSeenTime = 0;
};
}  // namespace commands