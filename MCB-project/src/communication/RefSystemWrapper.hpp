#pragma once

#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/drivers.hpp"

namespace communication {

using namespace tap::communication::serial;

class RefSystemWrapper {
public:
    RefSystemWrapper(tap::Drivers *drivers) : drivers(drivers) {}

    //temp not faked
    auto getRobotData() { return drivers->refSerial.getRobotData(); }
    auto getGameData() { return drivers->refSerial.getGameData(); }

#if defined(faked_ref)
    bool isConnected() { return true; }

    auto getGameType() {return tap::communication::serial::RefSerial::Rx::GameType::ROBOMASTER_RMUL_3V3;}

    auto getGameStage() { return tap::communication::serial::RefSerial::Rx::GameStage::PREMATCH;}

    auto getRemainingGameStageTime() {return 3;}

    auto getCurrentHP() {return 600;}

    // same
    uint16_t getRobotId(bool use100ForBlueTeam) { return use100ForBlueTeam ? (uint16_t)getRobotData().robotId : (uint16_t)getRobotData().robotId % 100; }

    bool isHealing() {return false;}

    bool isInReloadZone() {return false;}
    bool isInCenterZone() {return false;}
    bool doWeOccupyCenter() {return false;}
    bool doTheyOccupyCenter() {return false;}


#else


    // returns if ref serial is receiving data, so connected to ref sys, not necessarily a server
    bool isConnected() { return drivers->refSerial.getRefSerialReceivingData(); }

    // gets the current match type, used to know if in a match and if it is 3v3 or 1v1, shortcuts to this provided
    auto getGameType() { return drivers->refSerial.getGameData().gameType; }

    // gets the current game stage, assuming in a match
    auto getGameStage() { return drivers->refSerial.getGameData().gameStage; }

    auto getRemainingGameStageTime() { return drivers->refSerial.getGameData().stageTimeRemaining; }

    auto getCurrentHP() { return getRobotData().currentHp; }

    uint16_t getRobotId(bool use100ForBlueTeam) { return use100ForBlueTeam ? (uint16_t)getRobotData().robotId : (uint16_t)getRobotData().robotId % 100; }

    bool isHealing() { return getRobotData().robotBuffStatus.recoveryBuff > 0; }

    bool isInReloadZone() {
        return getRobotData().rfidStatus.any(
            tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::RESTORATION_ZONE | tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::EXCHANGE_ZONE);
    }

    bool isInCenterZone() { return getRobotData().rfidStatus.any(tap::communication::serial::RefSerial::Rx::RFIDActivationStatus::CENTRAL_BUFF); }

    bool doWeOccupyCenter() { return getGameData().eventData.siteData.any(tap::communication::serial::RefSerial::Rx::SiteData::CENTRAL_BUFF_OCCUPIED_TEAM); }
    bool doTheyOccupyCenter() { return getGameData().eventData.siteData.any(tap::communication::serial::RefSerial::Rx::SiteData::CENTRAL_BUFF_OCCUPIED_OPPONENT); }

#endif

    auto isOnBlueTeam() { return getRobotId(true) >= 100; }

    bool shooterHasPower() { !isConnected() || drivers->refSerial.getRobotData().robotPower.all(RefSerialData::Rx::RobotPower::SHOOTER_HAS_POWER); }
    bool drivetrainHasPower() { !isConnected() || drivers->refSerial.getRobotData().robotPower.all(RefSerialData::Rx::RobotPower::CHASSIS_HAS_POWER); }
    bool gimbalHasPower() { !isConnected() || drivers->refSerial.getRobotData().robotPower.all(RefSerialData::Rx::RobotPower::GIMBAL_HAS_POWER); }

    bool isIn1v1() { return isConnected() && getGameType() == RefSerialData::Rx::GameType::ROBOMASTER_RMUL_1V1; }
    bool isIn3v3() { return isConnected() && getGameType() == RefSerialData::Rx::GameType::ROBOMASTER_RMUL_3V3; }
    bool isConnectedToServer() { return isConnected() && getGameType() != RefSerialData::Rx::GameType::UNKNOWN; }

    bool isBefore15Sec() { return isConnected() && isConnectedToServer() && (getGameStage() == RefSerialData::Rx::GameStage::SETUP || getGameStage() == RefSerialData::Rx::GameStage::PREMATCH); }

    bool isInSetup() { return isConnected() && isConnectedToServer() && getGameStage() == RefSerialData::Rx::GameStage::SETUP; }
    bool isInPrematch() { return isConnected() && isConnectedToServer() && getGameStage() == RefSerialData::Rx::GameStage::PREMATCH; }
    bool isIn15Sec() { return isConnected() && isConnectedToServer() && getGameStage() == RefSerialData::Rx::GameStage::INITIALIZATION; }
    bool isIn5Sec() { return isConnected() && isConnectedToServer() && getGameStage() == RefSerialData::Rx::GameStage::COUNTDOWN; }
    bool isInGame() { return isConnected() && isConnectedToServer() && getGameStage() == RefSerialData::Rx::GameStage::IN_GAME; }
    bool isMatchOver() { return isConnected() && isConnectedToServer() && getGameStage() == RefSerialData::Rx::GameStage::END_GAME; }

private:
    tap::Drivers *drivers;
};

};  // namespace communication