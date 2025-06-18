
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

namespace src {
    
class ImuRecalibration {
public:

void requestRecalibration() {
    requestingRecalibration = true;
}

void cancelRequestRecalibration() {
    requestingRecalibration = false;
}

bool isRequestingRecalibration() {
    return requestingRecalibration;
}

void markAsRecalibrating() {
    requestingRecalibration = false;
    isRecalibrating = true;
}

bool getIsRecalibrating(){
    return isRecalibrating;
}

void markAsDoneRecalibrating() {
    isRecalibrating = false;
    isDoneRecalibrating = true;
}

bool getIsDoneRecalibrating() {
    return isDoneRecalibrating;
}

private:
bool requestingRecalibration = false; 
bool isRecalibrating = false; //occurs in 15 sec countdown
bool isDoneRecalibrating = false;

}; //class ImuRecalibration

class Drivers : public tap::Drivers {
public:
    Drivers() : tap::Drivers(), uart(this, tap::communication::serial::Uart::Uart1, true) {}

    communication::I2CCommunication i2c;
    communication::UARTCommunication uart;
    ImuRecalibration recal;
    





};  // class Drivers




}  // namespace src

#endif  // DRIVERS_HPP_

