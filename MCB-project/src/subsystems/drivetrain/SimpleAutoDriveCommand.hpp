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

    SimpleAutoDriveCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal, OdometrySubsystem* odo, TargetMode mode)
        : mode(mode),
          drivers(drivers),
          positionCommand(drivers, drive, gimbal, odo, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, 0.5f),
          odo(odo) {
        // always need to start at 0,0 because that is where the robot starts from
        // should be the reload/heal zone
        targets.push_back({{0.0f, 0.0f}, {0.0f, 0.0f}});

        addSubsystemRequirement(drive);
        stuckTimer.stop();
    }

    void initialize() {
        setupMap();
        isScheduled = true;
    }

    void execute() {
        // if we haven't self-disabled
        if (isScheduled) {
            // set direction
            setDirection();
            bool fasterSpinning = false;
            
            tap::communication::serial::RefSerial::Rx::RobotData robotData = drivers->refSerial.getRobotData();
            
            if(relocalizeState == RfidRelocalizeState::STUCK_WAITING_FOR_RESUPPLY || relocalizeState == RfidRelocalizeState::WAITING_FOR_RESUPPLY) {
                if(robotData.rfidStatus.all(tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::RESUPPLY_ZONE_OUTSIDE_EXCHANGE)){
                    //relocalize just x to a specific value
                    relocalizeState = RfidRelocalizeState::WAITING_FOR_CENTER;
                    odo->relocalizeTo(0.0f, odo->getY()); //here tune the x offset it relocalizes (the 0.0f) [If stuck on wall, make more negative (I think)]
                    if(timesRetriedRelocalize>=MAX_RELOCALIZE_TRIES){
                        //give up on movement
                        isScheduled = false;
                        // applies next time
                    }
                }
            }
            
            if(relocalizeState == RfidRelocalizeState::WAITING_FOR_CENTER){
                if(robotData.rfidStatus.all(tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::CENTRAL_BUFF)){
                    //allow relocalizing again
                    relocalizeState = RfidRelocalizeState::WAITING_FOR_RESUPPLY;
                    timesRetriedRelocalize=0;
                }
            }

            // if reached target, choose new target
            if (positionCommand.isFinished()) {
                bool allowAdvancing = true;
                int size = targets.size();

                // if in 3v3 match, wait for match to start before moving
                if (drivers->refSerial.getRefSerialReceivingData() && (drivers->refSerial.getGameData().gameType == RefSerialData::Rx::GameType::ROBOMASTER_RMUL_3V3)) {
                    allowAdvancing = allowAdvancing && drivers->refSerial.getGameData().gameStage == RefSerialData::Rx::GameStage::IN_GAME;
                }

                // if not moving, don't think you are stuck
                if (!allowAdvancing) stuckTimer.restart(STUCK_TIMER_AMOUNT);

                // going forwards
                if (allowAdvancing && direction == 1) {
                    stuckTimer.restart(STUCK_TIMER_AMOUNT);
                    if (targetIndex<size-1) {
                        // there is somewhere to go
                        targetIndex++;
                        if (needToApplyInitialPointChange) {
                            needToApplyInitialPointChange = false;
                            targets[0].first = changedInitialPoint;
                        }
                    } else {
                        // at endpoint (center)
                        fasterSpinning = true;
                    }
                }
                // going backwards
                if (allowAdvancing && direction == -1) {
                    stuckTimer.restart(STUCK_TIMER_AMOUNT);
                    if (targetIndex>0) {
                        // there is somewhere to go
                        targetIndex--;
                    } else {
                        // at endpoint (starting point/resupply)
                        fasterSpinning = true;
                    }
                }
            } 
            
            // haven't reached a target yet
            // set it
            float inputSpin = fasterSpinning ? SPIN_VELOCITY : commands::MoveToPositionCommand::MOVE_TO_POS_SPIN_VELO;
            positionCommand.targetPosition = {targets[targetIndex].first.first, targets[targetIndex].first.second, 0};
            positionCommand.inputVelocity = {direction * targets[targetIndex].second.first, direction * targets[targetIndex].second.second, inputSpin};

            // do movement
            positionCommand.execute();
            
            // if stuck
            if(stuckTimer.execute()){
                timesRetriedRelocalize++; //give up once at resupply, MAX_RELOCALIZE_TRIES
                direction = -1; //go backwards
                relocalizeState = RfidRelocalizeState::STUCK_WAITING_FOR_RESUPPLY;
            }
        } else { // !isScheduled
            // self disabled: spin fast in place
            positionCommand.drivetrain->setTargetTranslation({0, 0, SPIN_VELOCITY}, (positionCommand.drivetrain->angularVel < 6.0f || positionCommand.drivetrain->powerLimit >= 100.0f));
        }
    }

    bool isFinished() const override { return !drivers->remote.isConnected(); }

    bool getIsScheduled() { return isScheduled; }

    void end(bool cancel) override { isScheduled = false; }
    const char* getName() const override { return "simple auto drive command"; }

private:
    void setupMap() {
        bool flipXIfBlue = false;

        // 0,0 starting point added in constructor
        // these coordinates here are absolute, with the origin being where the robot was turned on from
        // positive x is right, positive y is forward
        // first pair is position, second pair is velocity (nonzero doesn't work well right now, so use 0, 0)
        switch (mode) {
            case TargetMode::TEST:
                targets.push_back({{0.0f, 1.0f}, {0.0f, 0.0f}});   // forward
                targets.push_back({{-1.0f, 1.0f}, {0.0f, 0.0f}});  // left
                targets.push_back({{-1.0f, 0.0f}, {0.0f, 0.0f}});  // back
                targets.push_back({{0.0f, 0.0f}, {0.0f, 0.0f}});   // right
                return;
            case TargetMode::PURDUE2V2:
                if (drivers->refSerial.isBlueTeam(drivers->refSerial.getRobotData().robotId)) {
                    targets.push_back({{-1.5f, 0.0f}, {0.0f, 0.0f}});
                    targets.push_back({{-1.5f, 1.5f}, {0.0f, 0.0f}});  // should be at center
                    targets.push_back({{0.5f, 3.5f}, {0.0f, 0.0f}});   // should be at center
                } else {
                    targets.push_back({{0.9f / 5, 0.0f / 5}, {1.5f, 0.0f}});
                    targets.push_back({{1.2f / 5, 0.0838f / 5}, {1.299f, 0.75f}});
                    targets.push_back({{1.419f / 5, 0.3f / 5}, {0.75f, 1.299f}});
                    targets.push_back({{1.5f / 5, 0.6f / 5}, {0.0f, 1.5f}});
                    targets.push_back({{1.5f / 5, 0.671f / 5}, {0.0f, 1.5f}});
                    targets.push_back({{1.461f / 5, 1.061f / 5}, {-0.29f, 1.47f}});
                    targets.push_back({{1.347f / 5, 1.437f / 5}, {-0.574f, 1.385f}});
                    targets.push_back({{1.162f / 5, 1.782f / 5}, {-0.833f, 1.247f}});
                    targets.push_back({{0.914f / 5, 2.086f / 5}, {-1.06f, 1.06f}});
                    targets.push_back({{-0.5f / 5, 3.5f / 5}, {0.0f, 0.0f}});  // should be at center
                }
                return;
            case TargetMode::ARCC_RAMP_PATH:
                flipXIfBlue = true;
                // coordinates for red team
                changedInitialPoint = {TOWARDS_ZONE_OFFSET, -TOWARDS_ZONE_OFFSET};
                targets.push_back({{-1.8330f, 0.595f}, {0.0f, 0.0f}});                                   // mostly left, some forward: before ramp
                targets.push_back({{-1.8330f, 4.060f}, {0.0f, 0.0f}});                                   // forward: across ramp
                targets.push_back({{TOWARDS_ZONE_OFFSET, 4.125f + TOWARDS_ZONE_OFFSET}, {0.0f, 0.0f}});  // mostly right, some forward: to center
                return;
            case TargetMode::ARCC_RAMP_PATH_HYPOTENUSE_ADJUSTED:
                flipXIfBlue = true;
                // coordinates for red team
                changedInitialPoint = {TOWARDS_ZONE_OFFSET, -TOWARDS_ZONE_OFFSET};
                targets.push_back({{-1.8330f, 0.595f}, {0.0f, 0.0f}});                                   // mostly left, some forward: before ramp
                targets.push_back({{-1.8330f, 4.872f}, {0.0f, 0.0f}});                                   // forward: across ramp (add 0.812)
                targets.push_back({{TOWARDS_ZONE_OFFSET, 4.937f + TOWARDS_ZONE_OFFSET}, {0.0f, 0.0f}});  // mostly right, some forward: to center (add 0.812)
                return;
            case TargetMode::ARCC_HALLWAY_PATH:
                flipXIfBlue = true;
                // coordinates for red team
                changedInitialPoint = {TOWARDS_ZONE_OFFSET, -TOWARDS_ZONE_OFFSET};
                targets.push_back({{-0.874f, 0.892f}, {0.0f, 0.0f}});                                             // left forward diagonal: before enter hallway
                targets.push_back({{-0.874f, 1.724f}, {0.0f, 0.0f}});                                             // forward: enter hallway
                targets.push_back({{0.693f, 1.724f}, {0.0f, 0.0f}});                                              // right: through hallway
                targets.push_back({{1.200f, 2.230f}, {0.0f, 0.0f}});                                              // forward right diagonal: leave hallway
                targets.push_back({{1.385f - TOWARDS_ZONE_OFFSET, 4.125f + TOWARDS_ZONE_OFFSET}, {0.0f, 0.0f}});  // forward: to center
                return;
            case TargetMode::ARCC_ROUGH_PATH:
                flipXIfBlue = true;
                // coordinates for red team
                changedInitialPoint = {-TOWARDS_ZONE_OFFSET, -TOWARDS_ZONE_OFFSET};
                targets.push_back({{2.236f, 0.0f}, {0.0f, 0.0f}});                                        // right forward diagonal: before wall
                targets.push_back({{2.236f, 1.224f}, {0.0f, 0.0f}});                                      // forward: past wall
                targets.push_back({{-TOWARDS_ZONE_OFFSET, 4.125f + TOWARDS_ZONE_OFFSET}, {0.0f, 0.0f}});  // left forward diagonal: to center
                return;
        }  // end switch

        // arcc map is mirrored across teams, mirror the x coordinates for blue team
        if (flipXIfBlue && drivers->refSerial.isBlueTeam(drivers->refSerial.getRobotData().robotId)) {
            for (unsigned int i = 0; i < targets.size(); i++) {
                targets[i].first.first = -targets[i].first.first;    // flip position x
                targets[i].second.first = -targets[i].second.first;  // flip velocity x (probably 0 though)
            }
        }
    }

    void setDirection() {
        if(relocalizeState==RfidRelocalizeState::STUCK_WAITING_FOR_RESUPPLY) return;
        
        switch (mode) {
            case TargetMode::TEST:  // change direction on hit
                if (drivers->refSerial.getRefSerialReceivingData()) {
                    static uint16_t oldHealth = drivers->refSerial.getRobotData().currentHp;
                    if (drivers->refSerial.getRobotData().currentHp != oldHealth) {
                        direction = -direction;
                        oldHealth = drivers->refSerial.getRobotData().currentHp;
                    }
                }
                return;
            default:                                                   // change direction on low health
                if (drivers->refSerial.getRefSerialReceivingData()) {  // 221 hp gate
                    float ratio = drivers->refSerial.getRobotData().currentHp * 1.0 / drivers->refSerial.getRobotData().maxHp;
                    if (ratio > 0.99) direction = 1;
                    if (ratio <= 0.5525) direction = -1;  // 0.5 or equal to try to avoid hero 1-shot-kills
                }
                return;
        }
    }

    int targetIndex = 0;  // index in targets
    int direction = 1;    // either 1 or -1
    bool isScheduled = false;

    bool needToApplyInitialPointChange = true;
    std::pair<float, float> changedInitialPoint{0.0f, 0.0f};

    static constexpr float TOWARDS_ZONE_OFFSET = 0.5f;  // meters, how far (x and y distance) into a zone (reload or center) to be. 0 would stay at a corner.

    TargetMode mode;
    src::Drivers* drivers;
    OdometrySubsystem* odo;
    tap::arch::MilliTimeout stuckTimer{};
    static constexpr int STUCK_TIMER_AMOUNT = 15 * 1000;  // ms, how long to wait between points

    MoveToPositionCommand positionCommand;

    std::vector<std::pair<
        std::pair<float, float>,  // position (x, y)
        std::pair<float, float>>>
        targets;  // velocity (x, y)
        
        
    enum class RfidRelocalizeState : uint8_t {
        WAITING_FOR_CENTER = 0, //don't relocalize at resupply until reach the center
        WAITING_FOR_RESUPPLY = 1,
        STUCK_WAITING_FOR_RESUPPLY = 2
    };
    
    RfidRelocalizeState relocalizeState = RfidRelocalizeState::WAITING_FOR_CENTER;
    
    int timesRetriedRelocalize = 0;
    
    static constexpr int MAX_RELOCALIZE_TRIES = 3;
};
}  // namespace commands
