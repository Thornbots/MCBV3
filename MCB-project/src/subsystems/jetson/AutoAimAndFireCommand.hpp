#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/gimbal/MouseMoveCommand.hpp"
#include "subsystems/flywheel/FlywheelSubsystem.hpp"
#include "subsystems/flywheel/FlywheelSubsystemConstants.hpp"
#include "subsystems/indexer/IndexerSubsystem.hpp"
#include "subsystems/jetson/JetsonSubsystem.hpp"
#include "subsystems/ui/objects/Reticle.hpp"

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
    // enum class AutoAimMode : uint8_t {
    //     SENTRY_MODE = 0,
    //     JOYSTICK_ASSIST = 1,
    //     MOUSE_ASSIST = 2
    // };

    AutoAimAndFireCommand(src::Drivers* drivers, GimbalSubsystem* gimbal, IndexerSubsystem* indexer, FlywheelSubsystem* flywheel, JetsonSubsystem* jetson, OdometrySubsystem* odo, AutoDriveCommand* adc, bool isManualControl);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "autoaim command"; }
    
    bool getIsScheduled();
    
    float targetYaw = 0;
    float targetPitch = 0;
    bool targeting = false;
    float deltaX = 0;
    float deltaY = 0;
    float deltaZ = 0;

    CvTarget cvTarget{};
    bool receivedCvTargetEver = false;

private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;
    IndexerSubsystem* indexer;
    FlywheelSubsystem* flywheel;
    JetsonSubsystem* jetson;
    OdometrySubsystem* odo;
    AutoDriveCommand* adc;
    bool isManualControl;

    int numCyclesForBurst = 0;
    static constexpr int CYCLES_UNTIL_BURST = 380; //cycles
    static constexpr float PATROL_SPEED = -0.0002; //rad/cycle
    static constexpr float BURST_AMOUNT = PATROL_SPEED; //rad/cycle, set to PATROL_SPEED to disable burst mode
    static constexpr float PATROL_PITCH = 0.05; // also used for turn to hit

    
    tap::arch::MilliTimeout cvTargetValidTimeout{};
    tap::arch::MilliTimeout startShotTimeout{}; //ideally we don't have to queue up multiple shots
    static constexpr int TARGET_VALID_TIME = 200; //ms, perhaps the new version of PERSISTANCE after the last shot
    static constexpr int FIRING_LATENCY_TIME = 80; //ms. When we tell the indexer to shoot, how long until that happens. Needs to be tested.
    

    
    // ---- turn-to-hit (sentry) ----
    // When patrolling and we take a hit, latch a world-yaw heading toward the hit source and hold
    // it briefly so CV has a chance to acquire the attacker before normal patrol resumes. This is
    // needed because jetson->getAngleToTurnForSentry() only reports the hit for a single cycle.
    bool turningToHit = false;
    float hitTargetYaw = 0.0f;                           // absolute world yaw (rad) held while facing a hit
    uint32_t hitTurnStartTime = 0;
    static constexpr uint32_t HIT_TURN_DURATION = 500;  // ms to face a hit before resuming patrol

    bool isScheduled = false;
};
}  // namespace commands