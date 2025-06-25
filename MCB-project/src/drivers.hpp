
/*
 * Copyright (c) 2020-2021 Thornbots
 *
 * This file is part of MCB.
 *
 * MCB is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MCB is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCB.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef DRIVERS_HPP_
#define DRIVERS_HPP_

#include "tap/drivers.hpp"

#include "communication/I2CCommunication.hpp"
#include "communication/UARTCommunication.hpp"
#include "communication/RefSystemWrapper.hpp"

#define PRINT(msg_to_send ...) ({\
    char str[BUFSIZ]; \
    sprintf(str, msg_to_send); \
    drivers->uart.sendMsg((uint8_t*)str, 2, strlen(str)); \
})

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
    AFTER_SECOND_CALIBRATION = 7
};

void requestRecalibration() {
    if(state==ImuRecalibrationState::AFTER_FIRST_CALIBRATION)
        state = ImuRecalibrationState::SECOND_CALIBRATION_REQUESTED;
}

void cancelRequestRecalibration() {
    if(state==ImuRecalibrationState::SECOND_CALIBRATION_REQUESTED)
        state = ImuRecalibrationState::AFTER_FIRST_CALIBRATION;
}

void setIsWaiting() {
    if(state==ImuRecalibrationState::SECOND_CALIBRATION_REQUESTED)
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
}

bool getIsImuReady() {
    return !getIsCalibrating();
}

bool isRequestingRecalibration() {
    return state==ImuRecalibrationState::SECOND_CALIBRATION_REQUESTED;
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

}; //class ImuRecalibration

class Drivers : public tap::Drivers {
public:
    Drivers() : tap::Drivers(), uart(this, tap::communication::serial::Uart::Uart1, true) {}

    communication::I2CCommunication i2c;
    communication::UARTCommunication uart;
    ImuRecalibration recal;
    communication::RefSystemWrapper refWrapper{this};
    





};  // class Drivers




}  // namespace src

#endif  // DRIVERS_HPP_

