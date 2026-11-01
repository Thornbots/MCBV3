#pragma once
#include "tap/communication/serial/ref_serial_data.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystemConstants.hpp"
#include "subsystems/drivetrain/MoveToPositionCommand.hpp"
#include "subsystems/odometry/OdometrySubsystem.hpp"

#include "drivers.hpp"

namespace commands {

using tap::communication::serial::Remote;
using namespace tap::communication::serial;

using namespace subsystems;

class SimpleAutoDriveCommand : public tap::control::Command {
public:
    enum class TargetMode : uint8_t {
        // a square
        TEST = 0,

        SPOOKY_SCRIM_2V2 = 6,

        // ARCC Season 2 3v3 Battlefield: square through the Starting Zone's 0.745 m
        // doorway, then spinning to just south of the Central Wall's tip.
        ARCC_S2_3V3 = 7
    };

    SimpleAutoDriveCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal, OdometrySubsystem* odo, TargetMode mode);

    void initialize();

    void execute();

    bool isFinished() const override;

    bool getIsScheduled();

    void end(bool cancel) override;
    const char* getName() const override { return "simple auto drive command"; }
    
private:
    friend class SimpleAutoDriveCommandTest;
    src::Drivers* drivers;
    OdometrySubsystem* odo;
    TargetMode mode;
    
    void setupMap();

    void setDirection();

    int targetIndex = 0;  // index in targets
    int direction = 1;    // either 1 or -1
    bool isScheduled = false;
    bool startCaptured = false;

    bool needToApplyInitialPointChange = true;
    std::pair<float, float> changedInitialPoint{0.0f, 0.0f};

    tap::arch::MilliTimeout stuckTimer{};
    static constexpr float WAYPOINT_TOLERANCE = 0.5f;  // meters, default arrival radius
    static constexpr float WALL_LEG_TOLERANCE = 0.2f;  // meters, prevent corner cutting
    static constexpr float DOORWAY_TOLERANCE = 0.05f;  // meters, line up before the doorway

    static constexpr int STUCK_TIMER_AMOUNT = 15 * 1000;  // ms, how long to wait between points

    MoveToPositionCommand positionCommand;

    struct Waypoint {
        std::pair<float, float> position;
        std::pair<float, float> velocity;
        float tolerance = WAYPOINT_TOLERANCE;
        // Drive the leg into this waypoint (either way) without spin, holding a quarter turn.
        bool square = false;
    };
    bool squareLeg() const;
    std::vector<Waypoint> targets;

};


}  // namespace commands
