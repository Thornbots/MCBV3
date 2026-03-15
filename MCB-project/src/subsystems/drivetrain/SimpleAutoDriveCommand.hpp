#pragma once
#include "drivers.hpp"


#include "subsystems/drivetrain/MoveToPositionCommand.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"


namespace commands {

using tap::communication::serial::Remote;
using namespace tap::communication::serial;


class SimpleAutoDriveCommand : public MoveToPositionCommand {
public:


    enum class TargetMode : uint8_t {
        TEST = 0,
        PURDUE2V2 = 1,
        ARCC = 2,
    };

    SimpleAutoDriveCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal, TargetMode mode)
        : MoveToPositionCommand(drivers, drive, gimbal, {0, 0, 0}), mode(mode){
        
        setupMap();
    }
    
    
    
    void execute() {
        // do movement
        MoveToPositionCommand::execute();
        
        // set direction
        setDirection();
        
        // if reached target, choose new target
        if(MoveToPositionCommand::isFinished()){
            bool allowAdvancing = true;
            int size = targets.size();
            
            // if in 3v3 match, wait for match to start before moving
            if(drivers->refSerial.getRefSerialReceivingData() && (drivers->refSerial.getGameData().gameType == RefSerialData::Rx::GameType::ROBOMASTER_RMUL_3V3)){
                allowAdvancing = allowAdvancing && drivers->refSerial.getGameData().gameStage == RefSerialData::Rx::GameStage::IN_GAME;
            }
            
            // going forwards if there is somewhere to go
            if(allowAdvancing && direction==1 && targetIndex<size-1){
                targetIndex++;
            }
            // going backwards if there is somewhere to go
            if(allowAdvancing && direction==-1 && targetIndex>0){
                targetIndex--;
            }
        }
        
        targetPosition = {targets[targetIndex].first, targets[targetIndex].second, 0};
    }
    
    
    bool isFinished() const override {
        return !drivers->remote.isConnected();
    }
    
private:

    void setupMap() {
        switch (mode) {
        case TargetMode::TEST:
            targets.push_back({0, 0.1});
            return;
        case TargetMode::PURDUE2V2:
            targets.push_back({ 0.0f, 0.0f}); //starting point (reload/heal zone) is 0,0
            targets.push_back({-2.0f, 1.8f});
            targets.push_back({-2.0f, 3.8f}); //should be at center
            return;
        case TargetMode::ARCC:
            return;
        }
    }

    void setDirection() {
        switch (mode) {
        case TargetMode::TEST:
            if(drivers->refSerial.getRefSerialReceivingData()) {
                static uint16_t oldHealth=0;
                if(drivers->refSerial.getRobotData().currentHp!=oldHealth){
                    direction = -direction;
                    oldHealth = drivers->refSerial.getRobotData().currentHp;
                }
            }
            return;
        default:
            if(drivers->refSerial.getRefSerialReceivingData()) {
                float ratio = drivers->refSerial.getRobotData().currentHp * 1.0 / drivers->refSerial.getRobotData().maxHp;
                if(ratio>0.9) direction=1;
                if(ratio<=0.5) direction=-1; //0.5 or equal to try to avoid hero 1-shot-kills. Could check if they have don't have a hero and use a different ratio
            }
            return;
        }
    }

    int targetIndex = 0; //index in targets
    int direction = 1; //either 1 or -1
    
    TargetMode mode;
    
    std::vector<std::pair<float, float>> targets; //don't need the rotation of Pose2d here, only need x and y
};
}