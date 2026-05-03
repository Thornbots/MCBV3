#pragma once
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "drivers.hpp"
#include "util/InputWrapper.hpp"
#include "util/trigger.hpp"

using namespace tap::control;
using namespace tap::communication::serial;
using namespace input;

#define TRIGGER(name, function) Trigger name{drivers, [this]() {return drivers->inputWrapper.function();}}

namespace robots
{
class ControlInterface
{
public:

    ControlInterface(src::Drivers* drivers) : drivers(drivers) {}
    //functions that all robots must have or at least share
    virtual void initialize() {}
    virtual void update() {}
    virtual void stopForImuRecal() {} //main calls this to stop the robot to recalibrate the imu
    virtual void resumeAfterImuRecal() {} //main calls this to after recalibrating the imu

    src::Drivers *drivers;

    TRIGGER(shootButton, isShootPressed);
    TRIGGER(unjamButton, isUnjamPressed);

    //controller driving
    TRIGGER(joystickDrive0, isRightSwitchUp);
    TRIGGER(joystickDrive1, isRightSwitchMid);
    TRIGGER(joystickDrive2, isRightSwitchDown);

    TRIGGER(joystickLook0, isLeftSwitchUp);
    TRIGGER(joystickLook1, isLeftSwitchMid);
    TRIGGER(joystickLook2, isLeftSwitchDown);

    #ifndef SENTRY
    TRIGGER(unjamKey, isUnjamKeyPressed);
    
    //toggle UI
    TRIGGER(toggleUIKey, isToggleUIPressed);

    //peeking
    TRIGGER(peekLeftButton, isPeekLeftPressed);
    TRIGGER(peekRightButton, isPeekRightPressed);

    Trigger peekNoneButton = !(peekLeftButton|peekRightButton);
    
    TRIGGER(scrollUp, isScrollUp);
    TRIGGER(scrollDown, isScrollDown);
    
    TRIGGER(autoAimKey, isAutoAimKeyPressed);
    TRIGGER(shootKey, isShootKeyPressed);

    TRIGGER(stopBeybladeKey, isStopBeybladePressed);
    TRIGGER(beybladeType1Key, isBeybladeSpinPressed); //most beyblade, checked in DrivetrainDriveCommand
    TRIGGER(beybladeType2Key, isBeybladeMovePressed); //most translation, checked in DrivetrainDriveCommand
    Trigger startBeybladeKey = beybladeType1Key | beybladeType2Key | scrollUp | scrollDown;

    Trigger stopFlywheelTrigger = unjamButton | unjamKey; //doesn't get added to the list of triggers, is special, during a match the only way to turn off flywheels is to turn off the remote
    #endif
};


}

//gaslights the compiler to think that RobotControl is one of these three based on the defines
#if defined(HERO)
#include "robots/hero/HeroControl.hpp"
using RobotControl = robots::HeroControl;

#elif defined(SENTRY)
#include "robots/sentry/SentryControl.hpp"
using RobotControl = robots::SentryControl;

#elif defined(INFANTRY)
#include "robots/infantry/InfantryControl.hpp"
using RobotControl = robots::InfantryControl;

#else //for old standard
#include "robots/infantry/InfantryControl.hpp"
using RobotControl = robots::InfantryControl;

#endif