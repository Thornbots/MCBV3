#pragma once
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/toggle_command_mapping.hpp"

#include "drivers.hpp"

using namespace tap::control;
using namespace tap::communication::serial;

namespace robots
{
class ControlInterface
{
public:

    ControlInterface() {}
    //functions that all robots must have or at least share
    virtual void initialize() {}
    virtual void update() {}
    virtual void stopForImuRecal() {} //main calls this to stop the robot to recalibrate the imu
    virtual void resumeAfterImuRecal() {} //main calls this to after recalibrating the imu
};


}

// Select the robot control implementation for the build target.
#if defined(HERO)
#include "robots/hero/HeroControl.hpp"
using RobotControl = robots::HeroControl;

#elif defined(SENTRY)
#include "robots/sentry/SentryControl.hpp"
using RobotControl = robots::SentryControl;

#elif defined(INFANTRY)
#include "robots/infantry/InfantryControl.hpp"
using RobotControl = robots::InfantryControl;

#elif defined(ENGINEER)
#include "robots/infantry/InfantryControl.hpp"
using RobotControl = robots::InfantryControl;

#else
#error "Select HERO, SENTRY, INFANTRY, or ENGINEER."
#endif
