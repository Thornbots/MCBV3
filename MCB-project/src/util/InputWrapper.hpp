#pragma once

#include "tap/drivers.hpp"
#include "tap/communication/serial/remote.hpp"
#include "util/trigger.hpp"

using tap::communication::serial::Remote;

namespace input {

class InputWrapper {
    private:
    tap::Drivers* drivers;

    public:
    InputWrapper(tap::Drivers* drivers) : drivers(drivers) {}

    bool isKeyPressed(Remote::Key key) {
        return drivers->remote.keyPressed(key);
    }

    bool isSwitchUp(Remote::Switch s) {
        return drivers->remote.getSwitch(s) == Remote::SwitchState::UP;
    }

    bool isSwitchMid(Remote::Switch s) {
        return drivers->remote.getSwitch(s) == Remote::SwitchState::MID;
    }

    bool isSwitchDown(Remote::Switch s) {
        return drivers->remote.getSwitch(s) == Remote::SwitchState::DOWN;
    }

    float getWheel() {
        return drivers->remote.getChannel(Remote::Channel::WHEEL);
    }

    float getChannel(Remote::Channel channel) {
        return drivers->remote.getChannel(channel);
    }

    bool isDriveForwardPressed() {
        return isKeyPressed(Remote::Key::W);
    }

    bool isDriveReversePressed() {
        return isKeyPressed(Remote::Key::S);
    }

    bool isStrafeLeftPressed() {
        return isKeyPressed(Remote::Key::A);
    }

    bool isStrafeRightPressed() {
        return isKeyPressed(Remote::Key::D);
    }

    float getDriveXAxis() {
        return getChannel(Remote::Channel::LEFT_HORIZONTAL);
    }

    float getDriveYAxis() {
        return getChannel(Remote::Channel::LEFT_VERTICAL);
    }

    bool isBoostPressed() {
        return isKeyPressed(Remote::Key::SHIFT);
    }

    bool isBeybladeMovePressed() {
        return isKeyPressed(Remote::Key::V);
    }

    bool isBeybladeSpinPressed() {
        return isKeyPressed(Remote::Key::C);
    }

    int16_t getMouseZ() {
        return drivers->remote.getMouseZ();
    }

    bool isScrollUp() {
        return getMouseZ() > 0;
    }

    bool isScrollDown() {
        return getMouseZ() < 0;
    }

    bool isShootPressed() {
        return getWheel() < -0.5;
    }

    bool isUnjamPressed() {
        return getWheel() > 0.5;
    }

    bool isUnjamKeyPressed() {
        return isKeyPressed(Remote::Key::Z);
    }

    bool isToggleUIPressed() {
        return isKeyPressed(Remote::Key::G);
    }

    bool isPeekLeftPressed() {
        return isKeyPressed(Remote::Key::Q);
    }

    bool isPeekRightPressed() {
        return isKeyPressed(Remote::Key::E);
    }

    bool isStopBeybladePressed() {
        return isKeyPressed(Remote::Key::X);
    }

    bool isAnyKeyPressed() {
        return (isKeyPressed(Remote::Key::Q) || isKeyPressed(Remote::Key::W) || isKeyPressed(Remote::Key::E) || isKeyPressed(Remote::Key::R) ||
                isKeyPressed(Remote::Key::A) || isKeyPressed(Remote::Key::S) || isKeyPressed(Remote::Key::D) || isKeyPressed(Remote::Key::F) ||
                isKeyPressed(Remote::Key::G) || isKeyPressed(Remote::Key::Z) || isKeyPressed(Remote::Key::X) || isKeyPressed(Remote::Key::C) ||
                isKeyPressed(Remote::Key::V) || isKeyPressed(Remote::Key::B));
    }

    bool isRightSwitchMid() {
        return isSwitchMid(Remote::Switch::RIGHT_SWITCH);
    }

    bool isRightSwitchDown() {
        return isSwitchDown(Remote::Switch::RIGHT_SWITCH);
    }

    bool isRightSwitchUp() {
        return isSwitchUp(Remote::Switch::RIGHT_SWITCH);
    }

    bool isLeftSwitchMid() {
        return isSwitchMid(Remote::Switch::LEFT_SWITCH);
    }

    bool isLeftSwitchDown() {
        return isSwitchDown(Remote::Switch::LEFT_SWITCH);
    }

    bool isLeftSwitchUp() {
        return isSwitchUp(Remote::Switch::LEFT_SWITCH);
    }

    bool isLeftMouseButtonPressed() {
        return drivers->remote.getMouseL();
    }

    bool isRightMouseButtonPressed() {
        return drivers->remote.getMouseR();
    }

    bool isAutoAimKeyPressed() {
        return isRightMouseButtonPressed();
    }

    bool isShootKeyPressed() {
        return isLeftMouseButtonPressed();
    }
};
}