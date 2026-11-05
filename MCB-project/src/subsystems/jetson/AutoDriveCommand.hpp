#pragma once

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/jetson/JetsonSubsystem.hpp"


#include "drivers.hpp"

namespace commands {
using subsystems::DrivetrainSubsystem;
using subsystems::GimbalSubsystem;
using subsystems::JetsonSubsystem;

using tap::communication::serial::Remote;


class AutoDriveCommand : public tap::control::Command {
public:
    AutoDriveCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal, JetsonSubsystem* jetson, OdometrySubsystem* odo)
        : drivers(drivers),
          jetson(jetson),
          drivetrain(drive),
          gimbal(gimbal),
          odo(odo) {
        addSubsystemRequirement(drive);
    }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "auto drive command"; }

    bool getIsScheduled();

    // Spin only once settled on the goal; travel holds the nearest field quarter
    // turn, because the 0.84 m spin sweep is wider than the Starting Zone's
    // 0.745 m doorway (sim world/arcc_s2_3v3/cad/NOTES.md).
    static constexpr float SPIN_START_RADIUS = 0.15f;  // m from the goal
    static constexpr float SPIN_STOP_RADIUS = 0.30f;   // m, hysteresis against chatter
    static constexpr float SPIN_VELOCITY = 9.0f;       // rad/s, unchanged from before


private:
    
    //for relocalization
    static constexpr float Y_DISTANCE_FROM_CENTER_THRESHOLD = 2.5; //meters, if jetson position is greater than this then allow relocalization
    static constexpr float DISTANCE_THRESHOLD = 0.5; //meters, if jetson position minus odo position is greater than this, relocalize

    float offsetX = 0;
    float offsetY = 0;

    src::Drivers* drivers;
    JetsonSubsystem* jetson;
    DrivetrainSubsystem* drivetrain;
    GimbalSubsystem* gimbal;
    OdometrySubsystem* odo;

    Pose2d targetPosition;
    Pose2d targetVelocity;

    bool isScheduled = false;
    bool hasNavGoal = false;
    bool spinning = false;

};
}  // namespace commands