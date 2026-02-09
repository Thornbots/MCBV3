#pragma once

#include "drivers.hpp"
#include "tap/communication/serial/remote.hpp"

using tap::communication::serial::Remote;

namespace InputMappings {
    Remote::Key forward = Remote::Key::W;
    Remote::Key reverse = Remote::Key::S;
    Remote::Key left = Remote::Key::A;
    Remote::Key right = Remote::Key::D;
    Remote::Key boost = Remote::Key::SHIFT;
    Remote::Key beybladeSpin = Remote::Key::C;
    Remote::Key beybladeMove = Remote::Key::V;

    Remote::Channel driveX = Remote::Channel::LEFT_HORIZONTAL;
    Remote::Channel driveY = Remote::Channel::LEFT_VERTICAL;
}

class InputWrapper {
    private:
    src::Drivers* drivers;

    public:
    InputWrapper(src::Drivers* drivers) : drivers(drivers){};

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
        return isKeyPressed(InputMappings::forward);
    }

    bool isDriveReversePressed() {
        return isKeyPressed(InputMappings::reverse);
    }

    bool isStrafeLeftPressed() {
        return isKeyPressed(InputMappings::left);
    }

    bool isStrafeRightPressed() {
        return isKeyPressed(InputMappings::right);
    }

    float getDriveXAxis() {
        return getChannel(InputMappings::driveX);
    }

    float getDriveYAxis() {
        return getChannel(InputMappings::driveY);
    }

    bool isBoostPressed() {
        return isKeyPressed(InputMappings::boost);
    }

    bool isBeybladeMovePressed() {
        return isKeyPressed(InputMappings::beybladeMove);
    }

    bool isBeybladeSpinPressed() {
        return isKeyPressed(InputMappings::beybladeSpin);
    }

    int16_t getMouseZ() {
        return drivers->remote.getMouseZ();
    }
};