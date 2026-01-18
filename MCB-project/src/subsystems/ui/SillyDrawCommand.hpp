#pragma once

#include "tap/control/command.hpp"

#include "subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "subsystems/flywheel/FlywheelSubsystem.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/indexer/IndexerSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "subsystems/servo/ServoSubsystem.hpp"

#include "util/ui/GraphicsContainer.hpp"
#include "objects/Video.hpp"
#include "objects/SupercapChargeIndicator.hpp"
#include "drivers.hpp"

namespace commands {
using subsystems::UISubsystem;

class SillyDrawCommand : public tap::control::Command, GraphicsContainer {
public:
    SillyDrawCommand(src::Drivers* drivers, UISubsystem* ui, GimbalSubsystem* gimbal, FlywheelSubsystem* flywheel, IndexerSubsystem* indexer, DrivetrainSubsystem* drivetrain, ServoSubsystem* servo)
        : drivers(drivers),
          ui(ui),
          gimbal(gimbal),
          flywheel(flywheel),
          indexer(indexer),
          drivetrain(drivetrain),
          servo(servo) {
        addSubsystemRequirement(ui);

        addGraphicsObject(&video);
        addGraphicsObject(&supercap);
    };

    void initialize() override { 
        ui->setTopLevelContainer(this); 
        
    };

    void execute() override {
        
        static bool wasPressed=false;
        if(drivers->remote.keyPressed(tap::communication::serial::Remote::Key::F)){
            if(!wasPressed){
                wasPressed=true;
                index++;
            }
        } else {
            wasPressed=false;
        }
        
        supercap.update();
        video.update(index*100);
        // logo doesn't need updating
    };

    //ui subsystem won't do anything until its top level container is set, so we are ok to add objects to the command in the constructor
    void end(bool) override { /*ui->setTopLevelContainer(nullptr);*/ };

    bool isFinished() const override { return false; };  // never done drawing ui

    const char* getName() const override { return "infantry ui draw command"; }

private:
    src::Drivers* drivers;
    UISubsystem* ui;
    GimbalSubsystem* gimbal;
    FlywheelSubsystem* flywheel;
    IndexerSubsystem* indexer;
    DrivetrainSubsystem* drivetrain;
    ServoSubsystem* servo;
    
    int index = 0;

    // add top level graphics objects here and in the constructor
    SupercapChargeIndicator supercap{drivetrain};
    Video video{};
};
}  // namespace commands