#pragma once

#include <optional>

#include "tap/control/command.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/jetson/JetsonSubsystem.hpp"


#include "drivers.hpp"

namespace commands {
using subsystems::DrivetrainSubsystem;
using subsystems::GimbalSubsystem;
using subsystems::JetsonSubsystem;

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

private:
    // Spin only once settled on the goal. Travel holds the nearest field quarter
    // turn, because the 0.84 m spin sweep is wider than the Starting Zone's
    // 0.745 m doorway (sim world/arcc_s2_3v3/cad/NOTES.md).
    static constexpr float SPIN_START_RADIUS = 0.025f; // m, inside the 0.05 m doorway waypoint tolerance
    static constexpr float SPIN_STOP_RADIUS = 0.30f;   // m, hysteresis against chatter
    static constexpr float SETTLED_SPEED = 0.10f;      // m/s
    static constexpr float SPIN_VELOCITY = 9.0f;       // rad/s
    static constexpr float TRAVEL_HEADING_ERROR = 0.06f; // rad
    static constexpr float TRAVEL_YAW_RATE = 0.35f;    // rad/s
    static constexpr float WAYPOINT_HANDOFF_RADIUS = 0.25f; // m, covers the doorway exit tolerance
    static constexpr float WAYPOINT_SETTLED_RADIUS = 0.01f; // m, settle on the next leg's centreline

    src::Drivers* drivers;
    JetsonSubsystem* jetson;
    DrivetrainSubsystem* drivetrain;
    GimbalSubsystem* gimbal;
    OdometrySubsystem* odo;

    Vector2d targetPosition;
    std::optional<Vector2d> previousGoal;
    std::optional<Vector2d> arrivalTarget;

    bool isScheduled = false;
    bool hasNavGoal = false;
    bool spinning = false;

};
}  // namespace commands
