#include "SimpleAutoDriveCommand.hpp"

namespace commands {

    SimpleAutoDriveCommand::SimpleAutoDriveCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal, OdometrySubsystem* odo, TargetMode mode)
        : drivers(drivers),
          odo(odo),
          mode(mode),
          positionCommand(drivers, drive, gimbal, odo, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, WAYPOINT_TOLERANCE) {
        // Slot zero is the boot point, captured on the first localized execute.
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
        isScheduled = true;
    }
    
    void SimpleAutoDriveCommand::execute() {
        if (!odo->isNavigationLocalized()) {
            stuckTimer.stop();
            positionCommand.drivetrain->stopMotors();
            return;
        }
        if (!startCaptured) {
            setupMap();
            // The boot point, not the current position: rescheduling mid-match keeps resupply.
            const std::pair<float, float> start{odo->getBootX(), odo->getBootY()};
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
            positionCommand.holdQuarterTurn = !fasterSpinning && squareLeg();

            // do movement
            positionCommand.execute();
            
            // if stuck
            if(stuckTimer.execute()){
                // self disable to spin fast in place
                isScheduled = false;
            }
        } else { // !isScheduled
            // self disabled: spin fast in place
            positionCommand.holdQuarterTurn = false;
            positionCommand.drivetrain->setTargetTranslation({0, 0, SPIN_VELOCITY}, (positionCommand.drivetrain->angularVel < 6.0f || positionCommand.drivetrain->powerLimit >= 100.0f));
        }
    }
    
    bool SimpleAutoDriveCommand::squareLeg() const {
        // Backwards, the leg (targetIndex + 1 -> targetIndex) is the reverse of the next one.
        const size_t i = targetIndex;
        return targets[i].square || (direction == -1 && i + 1 < targets.size() && targets[i + 1].square);
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
            case TargetMode::ARCC_S2_3V3:
                // The Side Wall end (y -0.255) and Side High Ground corner (y -1) leave a 0.745 m
                // doorway at x -4, narrower than the 0.84 m spin sweep: hold a quarter turn on
                // y -0.635 until clear (sim world/arcc_s2_3v3/cad/NOTES.md).
                targets.push_back({at(-5.25f, -0.635f), vel(0.0f, 0.0f), DOORWAY_TOLERANCE, true});  // north out of the zone
                targets.push_back({at(-2.40f, -0.635f), vel(0.0f, 0.0f), WALL_LEG_TOLERANCE, true});  // east through the doorway
                targets.push_back({at(-0.60f, -1.80f), vel(0.0f, 0.0f)});  // spinning, south of the Central Wall tip
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