#pragma once

#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/ui/ChassisOrientationIndicator.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace subsystems;

class AllRobotHealthNumbers : public GraphicsContainer {
public:
    AllRobotHealthNumbers(tap::Drivers* drivers) : drivers(drivers) {
        for(int i=0; i<6; i++){
            numbers[i].y = Y_POSITION;
            numbers[i].height = LINE_HEIGHT;

            this->addGraphicsObject(numbers + i);  // pointer math
        }

        for(int i=0; i<3; i++){
            numbers[i].color = UISubsystem::Color::PINK;
            numbers[i+3].color = UISubsystem::Color::CYAN;
        }
    }

    void update() {

        if (drivers->refSerial.getRefSerialReceivingData()) {
            RefSerialData::Rx::RobotHpData data = drivers->refSerial.getRobotData().allRobotHp;
            numbers[0].integer = data.red.sentry7;
            numbers[1].integer = data.red.standard3;
            numbers[2].integer = data.red.hero1;

            numbers[3].integer = data.blue.hero1;
            numbers[4].integer = data.blue.standard3;
            numbers[5].integer = data.blue.sentry7;

            //if in countdown, remove numbers because their robot icons disappear
            if(drivers->refSerial.getGameData().gameStage == RefSerialData::Rx::GameStage::COUNTDOWN){
                uint16_t x_i = INITIAL_X_OFFSET;
                uint16_t x_j = UISubsystem::SCREEN_WIDTH - INITIAL_X_OFFSET;
                for(int i=0; i<3; i++){
                    int j = 5-i; //if i is 0, j is 5; 1 and 4; 2 and 3

                    bool iIsHidden = numbers[i].integer == 0;
                    bool jIsHidden = numbers[j].integer == 0;

                    numbers[i].setHidden(iIsHidden);
                    numbers[j].setHidden(jIsHidden);

                    centerXs[i] = x_i;
                    centerXs[j] = x_j;
                    if(!iIsHidden){
                        x_i+=SUBSEQUENT_X_OFFSET;
                    }
                    if(!jIsHidden){
                        x_j-=SUBSEQUENT_X_OFFSET;
                    }
                }
            }

            

            for(int i=0; i<6; i++){
                numbers[i].calculateNumbers();

                if(drivers->refSerial.getRobotData().robotId == ids[i])
                    numbers[i].color = UISubsystem::Color::RED_AND_BLUE;
            }
            
            for(int i=0; i<3; i++){
                int j = 5-i; //if i is 0, j is 5; 1 and 4; 2 and 3

                numbers[i].x = centerXs[i] - numbers[i].width/2;
                numbers[j].x = centerXs[j] - numbers[j].width/2;
            }
        }
    }

private:
    tap::Drivers* drivers;

    static constexpr uint16_t Y_POSITION = 850; //pixels, all numbers at the same y level on screen
    static constexpr uint16_t LINE_HEIGHT = 20; //pixels, all numbers at the same size

    static constexpr uint16_t INITIAL_X_OFFSET = 180; //pixels, distance from left or right edge to center of number
    static constexpr uint16_t SUBSEQUENT_X_OFFSET = 120; //pixels, distance between centers of numbers

    //red sentry, red standard3, red hero, blue hero, blue standard3, blue sentry
    RefSerialData::RobotId ids[6] = {RefSerialData::RobotId::RED_SENTINEL, RefSerialData::RobotId::RED_SOLDIER_1, RefSerialData::RobotId::RED_HERO, RefSerialData::RobotId::BLUE_HERO, RefSerialData::RobotId::BLUE_SOLDIER_1, RefSerialData::RobotId::BLUE_SENTINEL};
    IntegerGraphic numbers[6];

    uint16_t centerXs[6];

};