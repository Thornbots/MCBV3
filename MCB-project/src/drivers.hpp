
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
#ifdef MCB_HOSTED
#include "hosted/sensors.hpp"
#endif

#include "communication/I2CCommunication.hpp"
#include "communication/UARTCommunication.hpp"

#include "util/HitTracker.hpp"
#include "util/ImuRecalibration.hpp"
#include "util/MappingMachine.hpp"

#define PRINT(msg_to_send ...) ({\
    char str[BUFSIZ]; \
    sprintf(str, msg_to_send); \
    drivers->uart.sendMsg((uint8_t*)str, 2, strlen(str)); \
})

namespace src {
    
class Drivers : public tap::Drivers {
public:
    Drivers() : tap::Drivers(), uart(this, tap::communication::serial::Uart::Uart1, true) {
#ifdef SENTRY
        uart.setPingHandler(&mapping, [](void* context, uint8_t number, uint32_t now) {
            static_cast<MappingMachine*>(context)->receivePing(number, now);
        });
#endif
    }

    communication::I2CCommunication i2c;
    communication::UARTCommunication uart;
    ImuRecalibration recal;
    MappingMachine mapping;
    HitTracker hitTracker{this}; 

    
    void executeCalibration() {
        this->bmi088.requestCalibration();
    }


    
    void adc1_pa6_init() {
#ifndef PLATFORM_HOSTED
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
#endif
    }

    uint16_t adc1_pa6_read() {
#ifdef PLATFORM_HOSTED
        return 0;
#else
        // Start conversion
        ADC1->CR2 |= ADC_CR2_SWSTART;

        // Wait for EOC
        while (!(ADC1->SR & ADC_SR_EOC));

        // Read result clears EOC
        return (uint16_t)ADC1->DR;
#endif
    }

};  // class Drivers




}  // namespace src

#endif  // DRIVERS_HPP_
