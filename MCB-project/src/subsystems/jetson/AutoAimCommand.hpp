#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/gimbal/MouseMoveCommand.hpp"
#include "subsystems/indexer/IndexerSubsystem.hpp"
#include "subsystems/jetson/JetsonSubsystem.hpp"


#include "drivers.hpp"

//this just does aiming, its up to the driver to shoot
namespace commands
{
using subsystems::GimbalSubsystem;
using subsystems::IndexerSubsystem;
using subsystems::JetsonSubsystem;
using tap::communication::serial::Remote;

class AutoAimCommand : public MouseMoveCommand
{
public:
    AutoAimCommand(src::Drivers* drivers, GimbalSubsystem* gimbal,JetsonSubsystem* cv)
        : MouseMoveCommand(drivers, gimbal),
          cv(cv)
    {
        //addSubsystemRequirement(gimbal); //MouseMoveCommand does this
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "autoaim command"; }
    


private:
    JetsonSubsystem* cv;
    int shoot = 0;
    bool isCalibrated = false;

    uint32_t lastSeenTime = 0;
    float pitch = 0.0f;
    float yawvel = 0.0f, pitchvel = 0.0f;
};
}  // namespace commands