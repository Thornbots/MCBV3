#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

//looks like a tiny square, roughly halfway between the center of the screen and the top right corner
//the color of it shows information
//black: can't get ref data, likely can't send it either, so you probably won't ever see black
//purple: occupied by both, not sure if the capturing allows both to capture or if it favors who was there first
//blue (cyan): occupied by blue team
//red (pink): occupied by red team
//white: neither team occupy it
class CapturePointStatusIndicator : public GraphicsContainer {
public:
    CapturePointStatusIndicator(tap::Drivers* drivers) : drivers(drivers) {
        addGraphicsObject(&square);
    }

    void update() {
        if(drivers->refSerial.getRefSerialReceivingData()){
            if(isOccupiedByOpponent() && isOccupiedByTeam()){
                square.color = UISubsystem::Color::PURPLISH_RED; //not sure if this will happen or not
            } else if (isOccupiedByTeam()) {
                square.color = drivers->refSerial.isBlueTeam(drivers->refSerial.getRobotData().robotId) ? UISubsystem::Color::CYAN : UISubsystem::Color::PINK;
            } else if (isOccupiedByOpponent()){
                square.color = drivers->refSerial.isBlueTeam(drivers->refSerial.getRobotData().robotId) ? UISubsystem::Color::PINK : UISubsystem::Color::CYAN;
            } else {
                square.color = UISubsystem::Color::WHITE;
            }
        } else {
            square.color = UISubsystem::Color::BLACK; //not sure if this will happen or not
        }
    }

private:
    tap::Drivers* drivers;

    bool isOccupiedByTeam(){
        //have to not not it to get it to be a bool
        return !!(drivers->refSerial.getGameData().eventData.siteData & tap::communication::serial::RefSerialData::Rx::SiteData::CENTRAL_BUFF_OCCUPIED_TEAM);
    }

    bool isOccupiedByOpponent(){
        return !!(drivers->refSerial.getGameData().eventData.siteData & tap::communication::serial::RefSerialData::Rx::SiteData::CENTRAL_BUFF_OCCUPIED_OPPONENT);
    }

    Line square{UISubsystem::Color::CYAN, 1230, 810, 1250, 810, 20};
};