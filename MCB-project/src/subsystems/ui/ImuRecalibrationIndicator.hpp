#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace subsystems;

//press ctrl r to schedule a imu recalibration just before the match starts
//press r to unschedule it
class ImuRecalibrationIndicator : public GraphicsContainer {
public:
    ImuRecalibrationIndicator(src::Drivers* drivers) : drivers(drivers) {
        addGraphicsObject(&line1);
        addGraphicsObject(&line2);
    }

    void update() {
        if (drivers->refSerial.getRefSerialReceivingData() && drivers->refSerial.getGameData().gameStage>RefSerialData::Rx::GameStage::COUNTDOWN){
            line1.hide();
            line2.hide();
            return;
        }

        if(expectingRecalibration && drivers->recal.getIsDoneRecalibrating()){
            line1.setString("Done Recalibrating");
            line2.setString("");
            line1.color = colorDoneRecalibrating;
            line2.color = colorDoneRecalibrating;
        } else if(drivers->recal.getIsRecalibrating()){
            line1.setString("Recalibrating");
            line2.setString("");
            line1.color = colorRecalibrating;
            line2.color = colorRecalibrating;
            expectingRecalibration = true;
        } else if (drivers->recal.isRequestingRecalibration()) {
            line1.setString("Imu recal scheduled");
            line2.setString("R to cancel");
            line1.color = colorScheduled;
            line2.color = colorScheduled;
            
            if(!ignoreKeyPresses && drivers->remote.keyPressed(Remote::Key::R)){
                drivers->recal.cancelRequestRecalibration();
                ignoreKeyPresses = true;
                update();
            }
        } else {
            line1.setString("CTRL+R to schedule");
            line2.setString("imu recalibration");
            line1.color = colorUnscheduled;
            line2.color = colorUnscheduled;

            if(!ignoreKeyPresses && drivers->remote.keyPressed(Remote::Key::CTRL) && drivers->remote.keyPressed(Remote::Key::R)){
                drivers->recal.requestRecalibration();
                ignoreKeyPresses = true;
                update();
            }

            expectingRecalibration = false;
        }

        if(!drivers->remote.keyPressed(Remote::Key::R))
            ignoreKeyPresses = false;
    }

private:
    src::Drivers* drivers;

    static constexpr uint16_t X_POSITION = 1680; //pixels, all numbers at the same y level on screen
    static constexpr uint16_t Y_POSITION = 610; //pixels, all numbers at the same y level on screen
    static constexpr uint16_t LINE_HEIGHT = 200; //pixels, this is a large number
    static constexpr uint16_t THICKENSS = 10; //pixels, this is a large number

    
    static constexpr UISubsystem::Color colorUnscheduled = UISubsystem::Color::CYAN;
    static constexpr UISubsystem::Color colorScheduled = UISubsystem::Color::ORANGE;
    static constexpr UISubsystem::Color colorRecalibrating = UISubsystem::Color::PURPLISH_RED;
    static constexpr UISubsystem::Color colorDoneRecalibrating = UISubsystem::Color::GREEN;

    StringGraphic line1{UISubsystem::Color::CYAN, "l1", 80, 600, 20, 2};
    StringGraphic line2{UISubsystem::Color::CYAN, "l2", 80, 550, 20, 2};

    bool ignoreKeyPresses = false;
    bool expectingRecalibration = false;
};