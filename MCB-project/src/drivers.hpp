
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

class Drivers : public tap::Drivers {
public:
    Drivers() : tap::Drivers(), uart(this, tap::communication::serial::Uart::Uart1, true) {}

    communication::I2CCommunication i2c;
    communication::UARTCommunication uart;
    ImuRecalibration recal;
    
    void executeCalibration() {
        this->bmi088.requestCalibration();
    }


    
    void adc1_pa6_init() {
        // 1. Enable clocks
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

        // 2. Configure PA6 as analog (MODER = 11)
        GPIOA->MODER |= (3U << (6 * 2));
        GPIOA->PUPDR &= ~(3U << (6 * 2)); // no pull-up/down

        // 3. ADC prescaler (common safe value: PCLK2 / 8)
        ADC->CCR &= ~(3U << 16);
        ADC->CCR |=  (1U << 16); // ADCPRE = 01 -> PCLK2/4 (or use 10 for /6, 11 for /8)

        // 4. ADC configuration
        ADC1->CR1 = 0;
        ADC1->CR2 = 0;

        // Right alignment
        ADC1->CR2 &= ~ADC_CR2_ALIGN;

        // 5. Single conversion mode (default)
        ADC1->CR2 &= ~ADC_CR2_CONT;

        // 6. Sampling time for channel 6 (PA6 -> IN6 in SMPR2)
        // choose 144 cycles for stability
        ADC1->SMPR2 &= ~(7U << (6 * 3));
        ADC1->SMPR2 |=  (7U << (6 * 3));

        // 7. Regular sequence length = 1 conversion
        ADC1->SQR1 &= ~(0xF << 20);

        // 8. Channel 6 as first conversion in sequence
        ADC1->SQR3 = 6;

        // 9. Enable ADC
        // Power on ADC
        ADC1->CR2 |= ADC_CR2_ADON;

        // mandatory stabilization delay
        for (volatile int i = 0; i < 10000; i++);

        // trigger a dummy conversion (VERY important)
        ADC1->CR2 |= ADC_CR2_SWSTART;
        while (!(ADC1->SR & ADC_SR_EOC));
        (void)ADC1->DR;
    }

    uint16_t adc1_pa6_read() {
        // Start conversion
        ADC1->CR2 |= ADC_CR2_SWSTART;

        // Wait for EOC
        while (!(ADC1->SR & ADC_SR_EOC));

        // Read result clears EOC
        return (uint16_t)ADC1->DR;
    }

};  // class Drivers




}  // namespace src

#endif  // DRIVERS_HPP_

