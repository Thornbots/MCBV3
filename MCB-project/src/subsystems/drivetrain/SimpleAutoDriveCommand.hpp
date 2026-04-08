#pragma once
#include "drivers.hpp"


#include "subsystems/drivetrain/MoveToPositionCommand.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"


namespace commands {

using tap::communication::serial::Remote;
using namespace tap::communication::serial;


class SimpleAutoDriveCommand : public tap::control::Command {
public:


    enum class TargetMode : uint8_t {
        TEST = 0,
        PURDUE2V2 = 1,
        ARCC = 2,
    };

    SimpleAutoDriveCommand(src::Drivers* drivers, DrivetrainSubsystem* drive, GimbalSubsystem* gimbal, TargetMode mode)
        : mode(mode), drivers(drivers),
        positionCommand(drivers, drive, gimbal, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f}, 0.5f)
        {
        
        setupMap();
        addSubsystemRequirement(drive);
    }
    
    void initialize() {

    }
    
    void execute() {
        // set direction
        setDirection();
        
        // if reached target, choose new target
        if(positionCommand.isFinished()){
        //if(positionCommand.isFinished()){
            drivers->leds.set(tap::gpio::Leds::Red, true);
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
        else {
            drivers->leds.set(tap::gpio::Leds::Red, false);
        }
        
        positionCommand.targetPosition = {targets[targetIndex].first.first, targets[targetIndex].first.second, 0};
        positionCommand.inputVelocity = {direction*targets[targetIndex].second.first, direction*targets[targetIndex].second.second, 0};

        //do movement
        positionCommand.execute();
        
    }
    
    
    bool isFinished() const override {
        return !drivers->remote.isConnected();
    }
    
    void end(bool cancel) override {
        //might fix sentry not being able to move after leaving auto drive
    }
    const char* getName() const override { return "simple auto drive command"; }
    
private:

    void setupMap() {
        switch (mode) {
        case TargetMode::TEST:
            targets.push_back({{0, 0}, {0, 0}});
            return;
        case TargetMode::PURDUE2V2:
            if(drivers->refSerial.isBlueTeam(drivers->refSerial.getRobotData().robotId))
            {
                targets.push_back({{0.0f, 0.0f}, {0.0f, 0.0f}}); //starting point (reload/heal zone) is 0,0
                targets.push_back({{-1.5f, 0.0f}, {0.0f, 0.0f}});
                targets.push_back({{-1.5f, 1.5f}, {0.0f, 0.0f}}); //should be at center
                targets.push_back({{0.5f, 3.5f}, {0.0f, 0.0f}}); //should be at center
            }
            else {
                targets.push_back({{0.0f, 0.0f}, {0.0f, 0.0f}}); //starting point (reload/heal zone) is 0,0
                targets.push_back({{0.9f, 0.0f}, {1.5f, 0.0f}});
                targets.push_back({{1.2f, 0.0838f}, {1.299f, 0.75f}});
                targets.push_back({{1.419f, 0.3f}, {0.75f, 1.299f}});
                targets.push_back({{1.5f, 0.6f}, {0.0f, 1.5f}});
                targets.push_back({{1.5f, 0.671f}, {0.0f, 1.5f}});
                targets.push_back({{1.461f, 1.061f}, {-0.29f, 1.47f}});
                targets.push_back({{1.347f, 1.437f}, {-0.574f, 1.385f}});
                targets.push_back({{1.162f, 1.782f}, {-0.833f, 1.247f}});
                targets.push_back({{0.914f, 2.086f}, {-1.06f, 1.06f}});
                targets.push_back({{-0.5f, 3.5f}, {0.0f, 0.0f}}); //should be at center
            }
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
            if(drivers->refSerial.getRefSerialReceivingData()) { //221 hp gate
                float ratio = drivers->refSerial.getRobotData().currentHp * 1.0 / drivers->refSerial.getRobotData().maxHp;
                if(ratio>0.99) direction=1;
                if(ratio<=0.5525) direction=-1; //0.5 or equal to try to avoid hero 1-shot-kills. Could check if they have don't have a hero and use a different ratio
            }
            return;
        }
    }

    int targetIndex = 0; //index in targets
    int direction = 1; //either 1 or -1
    
    TargetMode mode;
    src::Drivers* drivers;

    MoveToPositionCommand positionCommand;
    
    std::vector<std::pair<
    std::pair<float, float> //position
    , std::pair<float, float>>> targets; //velocity, don't need the rotation of Pose2d here, only need x and y
};
}