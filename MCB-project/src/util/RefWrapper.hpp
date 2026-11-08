#pragma once

#include "tap/drivers.hpp"

using namespace tap::communication::serial;

class RefWrapper {
public:

RefWrapper(tap::Drivers* drivers) : drivers(drivers){}

//returns true if we have the ref uart and we are in a match.
bool inGame(){
    return getGameStage()==RefSerialData::Rx::GameStage::IN_GAME;
}

bool inInitialization(){
    return getGameStage()==RefSerialData::Rx::GameStage::INITIALIZATION;
}

bool inCountdown(){
    return getGameStage()==RefSerialData::Rx::GameStage::COUNTDOWN;
}

bool isBeforeGame(){
    return getGameStage() < RefSerialData::Rx::GameStage::IN_GAME;
}

bool isInPrematch(){
    return getGameStage()==RefSerialData::Rx::GameStage::SETUP || getGameStage()==RefSerialData::Rx::GameStage::PREMATCH;
}

bool connectedToServer() {
    return drivers->refSerial.getRefSerialReceivingData() && drivers->refSerial.getGameData().gameType!=RefSerialData::Rx::GameType::UNKNOWN;
}

float getPowerLimit(){
    uint16_t minLimit = 240;
    if (isBeforeGame()) {
        minLimit = INITIAL_POWER_LIMIT_3V3;
    }
    return std::min(minLimit, drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);
}

bool refPoweringIndex() {
    return !drivers->refSerial.getRefSerialReceivingData() || drivers->refSerial.getRobotData().robotPower.any(RefSerialData::Rx::RobotPower::SHOOTER_HAS_POWER);
}

float getHeatLimit(){
    if(!drivers->refSerial.getRefSerialReceivingData()) return 100;
    return (float) drivers->refSerial.getRobotData().turret.heatLimit;
}

uint16_t getCoolingRate(){
    return drivers->refSerial.getRobotData().turret.coolingRate;
}

uint16_t getRedSentryHp(){
    return drivers->refSerial.getRobotData().allRobotHp.red.sentry7;
}

uint16_t getBlueSentryHp(){
    return drivers->refSerial.getRobotData().allRobotHp.blue.sentry7;
}

uint16_t getRedStandardHp(){
    return drivers->refSerial.getRobotData().allRobotHp.red.standard3;
}

uint16_t getBlueStandardHp(){
    return drivers->refSerial.getRobotData().allRobotHp.blue.standard3;
}

uint16_t getRedHeroHp(){
    return drivers->refSerial.getRobotData().allRobotHp.red.hero1;
}

uint16_t getBlueHeroHp(){
    return drivers->refSerial.getRobotData().allRobotHp.blue.hero1;
}

uint16_t getRedEngineerHp(){
    return drivers->refSerial.getRobotData().allRobotHp.red.engineer2;
}

uint16_t getBlueEngineerHp(){
    return drivers->refSerial.getRobotData().allRobotHp.blue.engineer2;
}

bool isBlue(){
    return drivers->refSerial.isBlueTeam(drivers->refSerial.getRobotData().robotId) && drivers->refSerial.getRefSerialReceivingData();
}

bool is1v1(){
    return drivers->refSerial.getRefSerialReceivingData() && drivers->refSerial.getGameData().gameType == RefSerialData::Rx::GameType::ROBOMASTER_RMUL_1V1;
}

uint16_t getPowerBuffer(){
    return drivers->refSerial.getRobotData().chassis.powerBuffer;
}

bool isInHealZone(){
    return drivers->refSerial.getRobotData().rfidStatus.all(tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::RESTORATION_ZONE) || drivers->refSerial.getRobotData().rfidStatus.all(tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::EXCHANGE_ZONE);
}

bool isAtCenter(){
    return drivers->refSerial.getRobotData().rfidStatus.all(tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::CENTRAL_BUFF);
}

uint16_t getCurrentHp(){
    return drivers->refSerial.getRobotData().currentHp;
}

bool isFullHp(){
    return getCurrentHp()==drivers->refSerial.getRobotData().maxHp;
}

RefSerialData::Rx::GameStage getGameStage(){
    if (drivers->refSerial.getRefSerialReceivingData() && 
        drivers->refSerial.getGameData().stageTimeRemaining==0 && 
        drivers->refSerial.getGameData().gameStage==RefSerialData::Rx::GameStage::COUNTDOWN){
        return RefSerialData::Rx::GameStage::IN_GAME;
    }
    return drivers->refSerial.getGameData().gameStage;
} 

uint16_t getStageTime(){
    if (drivers->refSerial.getRefSerialReceivingData() && 
        drivers->refSerial.getGameData().stageTimeRemaining==0 && 
        drivers->refSerial.getGameData().gameStage==RefSerialData::Rx::GameStage::COUNTDOWN){
            return 300;
        }
    return drivers->refSerial.getGameData().stageTimeRemaining;
}

uint8_t getRedTeamId(){
    return ((uint16_t) drivers->refSerial.getRobotData().robotId) % 100;
}

tap::communication::serial::RefSerialData::RobotId getId(){
    return drivers->refSerial.getRobotData().robotId;
}

bool isHealing(){
    return drivers->refSerial.getRobotData().robotBuffStatus.recoveryBuff>0;
}

bool teamHasCenter(){
    return drivers->refSerial.getGameData().eventData.siteData.any(tap::communication::serial::RefSerial::Rx::SiteData::CENTRAL_BUFF_OCCUPIED_TEAM);
}

bool opponentHasCenter(){
    return drivers->refSerial.getGameData().eventData.siteData.any(tap::communication::serial::RefSerial::Rx::SiteData::CENTRAL_BUFF_OCCUPIED_OPPONENT);
}

bool chassisHasPower(){
    return drivers->refSerial.getRobotData().robotPower.any(tap::communication::serial::RefSerial::Rx::RobotPower::CHASSIS_HAS_POWER);
}

bool gimbalHasPower(){
    return drivers->refSerial.getRobotData().robotPower.any(tap::communication::serial::RefSerial::Rx::RobotPower::GIMBAL_HAS_POWER);
}

int16_t getBulletsRemaining17(){
    return drivers->refSerial.getRobotData().turret.bulletsRemaining17;
}

int16_t getBulletsRemaining42(){
    return drivers->refSerial.getRobotData().turret.bulletsRemaining42;
}

private:

static constexpr float INITIAL_POWER_LIMIT_3V3=45;
tap::Drivers* drivers;
};