// Copyright 2020-2021 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

namespace src {

class ImuRecalibration {
public:

enum class ImuRecalibrationState : uint8_t {
    BEFORE_FIRST_CALIBRATION = 0,  
    FIRST_CALIBRATING = 1,         
    AFTER_FIRST_CALIBRATION = 2,   // recalibration is availiable
    SECOND_CALIBRATION_REQUESTED = 3, //if cancelled go back to AFTER_FIRST_CALIBRATION
    SECOND_CALIBRATION_WAITING_TO_START = 4, //when robot gets disabled and waiting for head to fall
    SECOND_CALIBRATION_JUST_BEFORE_START = 5, 
    SECOND_CALIBRATING = 6,
    AFTER_SECOND_CALIBRATION = 7,
    FORCE_CALIBRATION = 8
};

void requestRecalibration() {
    if(state==ImuRecalibrationState::AFTER_FIRST_CALIBRATION) {
        pendingScheduledRecalibration = false;
        state = ImuRecalibrationState::SECOND_CALIBRATION_REQUESTED;
    } else {
        pendingScheduledRecalibration = true;
    }
}

void forceCalibration() {
    state=ImuRecalibrationState::FORCE_CALIBRATION;
}

void cancelRequestRecalibration() {
    if(state==ImuRecalibrationState::SECOND_CALIBRATION_REQUESTED)
        state = ImuRecalibrationState::AFTER_FIRST_CALIBRATION;
}

void setIsWaiting() {
    if(state==ImuRecalibrationState::SECOND_CALIBRATION_REQUESTED || state==ImuRecalibrationState::FORCE_CALIBRATION)
        state = ImuRecalibrationState::SECOND_CALIBRATION_WAITING_TO_START;
}

void setIsFirstCalibrating() {
    if(state==ImuRecalibrationState::BEFORE_FIRST_CALIBRATION)
        state = ImuRecalibrationState::FIRST_CALIBRATING;
}

void setJustBeforeSecondCalibrating() {
    if(state==ImuRecalibrationState::SECOND_CALIBRATION_WAITING_TO_START)
        state = ImuRecalibrationState::SECOND_CALIBRATION_JUST_BEFORE_START;
}

void setIsSecondCalibrating() {
    if(state==ImuRecalibrationState::SECOND_CALIBRATION_JUST_BEFORE_START)
        state = ImuRecalibrationState::SECOND_CALIBRATING;
}

bool getIsCalibrating() {
    return state==ImuRecalibrationState::FIRST_CALIBRATING || state==ImuRecalibrationState::SECOND_CALIBRATING;
}

void setIsDoneCalibrating() {
    if(state==ImuRecalibrationState::FIRST_CALIBRATING)
        state = ImuRecalibrationState::AFTER_FIRST_CALIBRATION;

    if(state==ImuRecalibrationState::SECOND_CALIBRATING)
        state = ImuRecalibrationState::AFTER_SECOND_CALIBRATION;
        
    if (pendingScheduledRecalibration)
        requestRecalibration();
}

bool getIsImuReady() {
    return !getIsCalibrating();
}

bool isRequestingRecalibration() {
    return state==ImuRecalibrationState::SECOND_CALIBRATION_REQUESTED;
}

bool isForcingRecalibration() {
    return state==ImuRecalibrationState::FORCE_CALIBRATION;
}

bool isAfterSecondCalibration() {
    return state==ImuRecalibrationState::AFTER_SECOND_CALIBRATION;
}

//when in a best of 3 (or best of 2) game, we allow third and fourth calibrations by going back to first calibration
void allowAnotherRecalibration() {
    if(state==ImuRecalibrationState::AFTER_SECOND_CALIBRATION)
        state = ImuRecalibrationState::AFTER_FIRST_CALIBRATION;
}

ImuRecalibrationState getState() {
    return state;
}


private:
ImuRecalibrationState state = ImuRecalibrationState::BEFORE_FIRST_CALIBRATION;
bool pendingScheduledRecalibration = false;

}; //class ImuRecalibration

}  // namespace src
