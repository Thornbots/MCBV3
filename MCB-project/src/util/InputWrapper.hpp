#pragma once

#include "tap/drivers.hpp"
#include "tap/communication/serial/remote.hpp"

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
};
}