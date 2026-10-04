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

        // 2v2 map at purdue midwest conference
        PURDUE2V2 = 1,

        // longest, interferes with where hero will probably be, but no rough or ramps
        ARCC_HALLWAY_PATH = 2,

        // shortest, odo might lose a lot of accuracy on the rough.
        ARCC_ROUGH_PATH = 3,

        // odo will read wrong on the sloped ramp. This path pretends the ramp isn't there, and hopes
        // jetson relocalization (lidar and/or april tags) corrects the odo.
        ARCC_RAMP_PATH = 4,

        // odo will read wrong on the sloped ramp. This path pretends the ramp is longer, to account for
        // measuring the hypotenuses of the triangluar ramps with the odo pods.
        // Won't work well with jetson relocalization
        ARCC_RAMP_PATH_HYPOTENUSE_ADJUSTED = 5
    };

    SimpleAutoDriveCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal, OdometrySubsystem* odo, TargetMode mode);

    void initialize();

    void execute();

    bool isFinished() const override;

    bool getIsScheduled();

    void end(bool cancel) override;
    const char* getName() const override { return "simple auto drive command"; }
    
private:
    src::Drivers* drivers;
    OdometrySubsystem* odo;
    TargetMode mode;
    
    void setupMap();

    void setDirection();

    int targetIndex = 0;  // index in targets
    int direction = 1;    // either 1 or -1
    bool isScheduled = false;

    bool needToApplyInitialPointChange = true;
    std::pair<float, float> changedInitialPoint{0.0f, 0.0f};

    static constexpr float TOWARDS_ZONE_OFFSET = 0.5f;  // meters, how far (x and y distance) into a zone (reload or center) to be. 0 would stay at a corner.

    tap::arch::MilliTimeout stuckTimer{};
    static constexpr int STUCK_TIMER_AMOUNT = 15 * 1000;  // ms, how long to wait between points

    MoveToPositionCommand positionCommand;

    std::vector<std::pair<
        std::pair<float, float>,  // position (x, y)
        std::pair<float, float>>>
        targets;  // velocity (x, y)
};


}  // namespace commands
