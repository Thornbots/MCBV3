#include "SimpleAutoDriveCommand.hpp"

namespace commands {

    SimpleAutoDriveCommand::SimpleAutoDriveCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal, OdometrySubsystem* odo, TargetMode mode)
        : drivers(drivers),
          odo(odo),
          mode(mode),
          positionCommand(drivers, drive, gimbal, odo, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, WAYPOINT_TOLERANCE) {
        // Slot zero is captured from localized navigation on the first execute.
        targets.push_back({{0.0f, 0.0f}, {0.0f, 0.0f}});

        addSubsystemRequirement(drive);
        stuckTimer.stop();
    }
    
    void SimpleAutoDriveCommand::initialize() {
        targets.clear();
        targets.push_back({{0, 0}, {0, 0}});
        targetIndex = 0;
        direction = 1;
        startCaptured = false;
        needToApplyInitialPointChange = false;
        stuckTimer.stop();
        setupMap();
        isScheduled = true;
    }
    
    void SimpleAutoDriveCommand::execute() {
        if (!odo->isNavigationLocalized()) {
            stuckTimer.stop();
            positionCommand.drivetrain->stopMotors();
            return;
        }
        if (!startCaptured) {
            const std::pair<float, float> start{odo->getX(), odo->getY()};
            targets[0].position = start;
            if (mode == TargetMode::TEST) {
                for (size_t i = 1; i < targets.size(); ++i) {
                    targets[i].position.first += start.first;
                    targets[i].position.second += start.second;
                }
            }
            positionCommand.targetPosition = {start.first, start.second, 0};
            startCaptured = true;
        }
        // if we haven't self-disabled
        if (isScheduled) {
            // set direction
            setDirection();
            bool fasterSpinning = false;
            
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
                            targets[0].position = changedInitialPoint;
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
            positionCommand.targetPosition = {targets[targetIndex].position.first, targets[targetIndex].position.second, 0};
            positionCommand.inputVelocity = {direction * targets[targetIndex].velocity.first, direction * targets[targetIndex].velocity.second, inputSpin};

            positionCommand.setTolerance(targets[targetIndex].tolerance);

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
        // Absolute red field coordinates; blue reflects x for positions and velocities.
        auto at = [isBlue](float x, float y) {
            return std::pair<float, float>{isBlue ? -x : x, y};
        };
        auto vel = at;
        switch (mode) {
            case TargetMode::TEST:
                targets.push_back({at(1.0f, 0.0f), vel(0.0f, 0.0f)});     // forward
                targets.push_back({at(1.0f, 1.0f), vel(0.0f, 0.0f)});   // left
                targets.push_back({at(0.0f, 1.0f), vel(0.0f, 0.0f)});   // back
                targets.push_back({at(0.0f, 0.0f), vel(0.0f, 0.0f)});     // right
                return;
            case TargetMode::ARCC_RAMP_PATH:
                // coordinates for red team
                needToApplyInitialPointChange = true;
                changedInitialPoint = at(-5.125f, -0.5f);
                targets.push_back({at(-4.030f, 1.8330f), vel(0.0f, 0.0f)});                                   // mostly left, some forward: before ramp
                targets.push_back({at(-0.565f, 1.8330f), vel(0.0f, 0.0f)});                                   // forward: across ramp
                targets.push_back({at(0.0f, -0.5f), vel(0.0f, 0.0f)});  // mostly right, some forward: to center
                return;
            case TargetMode::ARCC_RAMP_PATH_HYPOTENUSE_ADJUSTED:
                // coordinates for red team
                needToApplyInitialPointChange = true;
                changedInitialPoint = at(-5.125f, -0.5f);
                targets.push_back({at(-4.030f, 1.8330f), vel(0.0f, 0.0f)});                                   // mostly left, some forward: before ramp
                targets.push_back({at(0.247f, 1.8330f), vel(0.0f, 0.0f)});                                   // forward: across ramp (add 0.812)
                targets.push_back({at(0.812f, -0.5f), vel(0.0f, 0.0f)});  // mostly right, some forward: to center (add 0.812)
                return;
            case TargetMode::ARCC_HALLWAY_PATH:
                // coordinates for red team
                needToApplyInitialPointChange = true;
                changedInitialPoint = at(-5.125f, -0.5f);
                targets.push_back({at(-3.733f, 0.874f), vel(0.0f, 0.0f)});                                               // left forward diagonal: before enter hallway
                targets.push_back({at(-2.901f, 0.874f), vel(0.0f, 0.0f)});                                               // forward: enter hallway
                targets.push_back({at(-2.901f, -0.693f), vel(0.0f, 0.0f)});                                                // right: through hallway
                targets.push_back({at(-2.395f, -1.200f), vel(0.0f, 0.0f)});                                                // forward right diagonal: leave hallway
                targets.push_back({at(0.0f, -0.885f), vel(0.0f, 0.0f)});  // forward: to center
                return;
            case TargetMode::ARCC_ROUGH_PATH:
                // coordinates for red team
                needToApplyInitialPointChange = true;
                changedInitialPoint = at(-5.125f, 0.5f);
                // the wall leg keeps the spinning chassis (0.42 m sweep) 0.1 m clear of the wall end, 1.81 m right of the start
                targets.push_back({at(-4.125f, -2.5f), vel(0.0f, 0.0f), WALL_LEG_TOLERANCE});                                          // right forward diagonal: before wall
                targets.push_back({at(-3.225f, -2.5f), vel(0.0f, 0.0f), WALL_LEG_TOLERANCE});                                          // forward: past wall
                targets.push_back({at(0.0f, 0.5f), vel(0.0f, 0.0f)});  // left forward diagonal: to center
                return;
            case TargetMode::SPOOKY_SCRIM_2V2:
                targets.push_back({at(-3.50f, -1.75f), vel(0, 0)});
                targets.push_back({at(-2.19f, -1.76f), vel(0, 0)});
                targets.push_back({at(0, 0), vel(0, 0)});
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