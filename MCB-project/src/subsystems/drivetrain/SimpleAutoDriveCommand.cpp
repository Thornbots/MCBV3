#include "SimpleAutoDriveCommand.hpp"

namespace commands {

    SimpleAutoDriveCommand::SimpleAutoDriveCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal, OdometrySubsystem* odo, TargetMode mode)
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
    
    void SimpleAutoDriveCommand::initialize() {
        setupMap();
        isScheduled = true;
    }
    
    void SimpleAutoDriveCommand::execute() {
        // if we haven't self-disabled
        if (isScheduled) {
            // set direction
            setDirection();
            bool fasterSpinning = false;
            
            tap::communication::serial::RefSerial::Rx::RobotData robotData = drivers->refSerial.getRobotData();
            tap::communication::serial::RefSerial::Rx::GameData gameData = drivers->refSerial.getGameData();

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
                        if(gameData.gameStage == RefSerialData::Rx::GameStage::IN_GAME)  //comment this out to test withoug a server
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
                // self disable to spin fast in place
                isScheduled = false;
            }
        } else { // !isScheduled
            // self disabled: spin fast in place
            positionCommand.drivetrain->setTargetTranslation({0, 0, SPIN_VELOCITY}, (positionCommand.drivetrain->angularVel < 6.0f || positionCommand.drivetrain->powerLimit >= 100.0f));
        }
    }
    
    bool SimpleAutoDriveCommand::isFinished() const { return !drivers->remote.isConnected(); }
    
    bool SimpleAutoDriveCommand::getIsScheduled() { return isScheduled; }
    
    void SimpleAutoDriveCommand::end(bool) { isScheduled = false; }
    
    void SimpleAutoDriveCommand::setupMap() {
        bool isBlue = drivers->refSerial.isBlueTeam(drivers->refSerial.getRobotData().robotId);
        int m = isBlue ? -1 : 1;

        // 0,0 starting point added in constructor
        // these coordinates here are absolute, with the origin being where the robot was turned on from
        // REP-105: positive x is forward, positive y is left
        // first pair is position, second pair is velocity (nonzero doesn't work well right now, so use 0, 0)
        switch (mode) {
            case TargetMode::TEST:
                targets.push_back({{1.0f, 0.0f}, {0.0f, 0.0f}});     // forward
                targets.push_back({{1.0f, m*1.0f}, {0.0f, 0.0f}});   // left
                targets.push_back({{0.0f, m*1.0f}, {0.0f, 0.0f}});   // back
                targets.push_back({{0.0f, 0.0f}, {0.0f, 0.0f}});     // right
                return;
            case TargetMode::PURDUE2V2:
                if (drivers->refSerial.isBlueTeam(drivers->refSerial.getRobotData().robotId)) {
                    targets.push_back({{0.0f, 1.5f}, {0.0f, 0.0f}});
                    targets.push_back({{1.5f, 1.5f}, {0.0f, 0.0f}});
                    targets.push_back({{3.5f, -0.5f}, {0.0f, 0.0f}});   // should be at center
                } else {
                    targets.push_back({{0.0f / 5, -0.9f / 5}, {0.0f, -1.5f}});
                    targets.push_back({{0.0838f / 5, -1.2f / 5}, {0.75f, -1.299f}});
                    targets.push_back({{0.3f / 5, -1.419f / 5}, {1.299f, -0.75f}});
                    targets.push_back({{0.6f / 5, -1.5f / 5}, {1.5f, 0.0f}});
                    targets.push_back({{0.671f / 5, -1.5f / 5}, {1.5f, 0.0f}});
                    targets.push_back({{1.061f / 5, -1.461f / 5}, {1.47f, 0.29f}});
                    targets.push_back({{1.437f / 5, -1.347f / 5}, {1.385f, 0.574f}});
                    targets.push_back({{1.782f / 5, -1.162f / 5}, {1.247f, 0.833f}});
                    targets.push_back({{2.086f / 5, -0.914f / 5}, {1.06f, 1.06f}});
                    targets.push_back({{3.5f / 5, 0.5f / 5}, {0.0f, 0.0f}});  // should be at center
                }
                return;
            case TargetMode::ARCC_RAMP_PATH:
                // coordinates for red team
                changedInitialPoint = {-TOWARDS_ZONE_OFFSET, -m*TOWARDS_ZONE_OFFSET};
                targets.push_back({{0.595f, m*1.8330f}, {0.0f, 0.0f}});                                   // mostly left, some forward: before ramp
                targets.push_back({{4.060f, m*1.8330f}, {0.0f, 0.0f}});                                   // forward: across ramp
                targets.push_back({{4.125f + TOWARDS_ZONE_OFFSET, -m*TOWARDS_ZONE_OFFSET}, {0.0f, 0.0f}});  // mostly right, some forward: to center
                return;
            case TargetMode::ARCC_RAMP_PATH_HYPOTENUSE_ADJUSTED:
                // coordinates for red team
                changedInitialPoint = {-TOWARDS_ZONE_OFFSET, -m*TOWARDS_ZONE_OFFSET};
                targets.push_back({{0.595f, m*1.8330f}, {0.0f, 0.0f}});                                   // mostly left, some forward: before ramp
                targets.push_back({{4.872f, m*1.8330f}, {0.0f, 0.0f}});                                   // forward: across ramp (add 0.812)
                targets.push_back({{4.937f + TOWARDS_ZONE_OFFSET, -m*TOWARDS_ZONE_OFFSET}, {0.0f, 0.0f}});  // mostly right, some forward: to center (add 0.812)
                return;
            case TargetMode::ARCC_HALLWAY_PATH:
                // coordinates for red team
                changedInitialPoint = {-TOWARDS_ZONE_OFFSET, -m*TOWARDS_ZONE_OFFSET};
                targets.push_back({{0.892f, m*0.874f}, {0.0f, 0.0f}});                                               // left forward diagonal: before enter hallway
                targets.push_back({{1.724f, m*0.874f}, {0.0f, 0.0f}});                                               // forward: enter hallway
                targets.push_back({{1.724f, -m*0.693f}, {0.0f, 0.0f}});                                                // right: through hallway
                targets.push_back({{2.230f, -m*1.200f}, {0.0f, 0.0f}});                                                // forward right diagonal: leave hallway
                targets.push_back({{4.125f + TOWARDS_ZONE_OFFSET, -m*(1.385f - TOWARDS_ZONE_OFFSET)}, {0.0f, 0.0f}});  // forward: to center
                return;
            case TargetMode::ARCC_ROUGH_PATH:
                // coordinates for red team
                changedInitialPoint = {-TOWARDS_ZONE_OFFSET, m*TOWARDS_ZONE_OFFSET};
                targets.push_back({{0.5f, -m*2.236f}, {0.0f, 0.0f}});                                        // right forward diagonal: before wall
                targets.push_back({{1.224f, -m*2.236f}, {0.0f, 0.0f}});                                      // forward: past wall
                targets.push_back({{4.125f + TOWARDS_ZONE_OFFSET, m*TOWARDS_ZONE_OFFSET}, {0.0f, 0.0f}});  // left forward diagonal: to center
                return;
        }  // end switch
    }

    void SimpleAutoDriveCommand::setDirection() {
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
}