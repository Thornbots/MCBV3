#pragma once

#include <cstdint>

#include "mt6701.hpp"
#include "LocalizationPico.hpp"

namespace communication {

class I2CCommunication {
public:
    void initialize() {
        __HAL_RCC_GPIOF_CLK_ENABLE();
        __HAL_RCC_I2C2_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct{};
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        hi2c2.Instance = I2C2;
        hi2c2.Init.ClockSpeed = 400000;
        hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
        hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
        hi2c2.Init.OwnAddress1 = 0;
        hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
        hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
        hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

        HAL_I2C_Init(&hi2c2);

        HAL_NVIC_SetPriority(I2C2_EV_IRQn, 10, 0);
        HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
        HAL_NVIC_SetPriority(I2C2_ER_IRQn, 10, 0);
        HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);

        encoder.init(&hi2c2);
        odom.init(&hi2c2);
    }

    void refresh() {
        encoder.startRead();

#if defined(SENTRY)
        odom.startRead();
#endif
    }

    void handleRxComplete(I2C_HandleTypeDef *hi2c) {
        encoder.handleRxComplete(hi2c);
        odom.handleRxComplete(hi2c);
    }

    I2C_HandleTypeDef hi2c2;

    MT6701 encoder;
    LocalizationPico odom;
};

} // namespace communication