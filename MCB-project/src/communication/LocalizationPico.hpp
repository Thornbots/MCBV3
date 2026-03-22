#pragma once
extern "C" {
#include "i2c/inc/stm32f4xx_hal.h"
}
#include <cstdint>

namespace communication {

class LocalizationPico {
public:
    void init(I2C_HandleTypeDef* bus) {
        hi2c = bus;
    }

    void startRead() {
        if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) return;

        reg = 0x00;
        HAL_I2C_Mem_Read_IT(
            hi2c,
            ADDRESS << 1,
            reg,
            I2C_MEMADD_SIZE_8BIT,
            reinterpret_cast<uint8_t*>(buffer),
            sizeof(buffer)
        );
    }

    void handleRxComplete(I2C_HandleTypeDef* bus) {
        if (bus != hi2c) return;

        odom_x = buffer[0];
        odom_y = buffer[1];
        odom_x_vel = buffer[2];
        odom_y_vel = buffer[3];
    }

    float getX() const { return odom_x + offsetX; }
    float getY() const { return odom_y + offsetY; }
    float getXVel() const { return odom_x_vel; }
    float getYVel() const { return odom_y_vel; }

    void setOffset(float x, float y) {
        offsetX = x;
        offsetY = y;
    }

private:
    I2C_HandleTypeDef* hi2c = nullptr;

    static constexpr uint8_t ADDRESS = 0x55;

    uint8_t reg;

    float buffer[4]{};

    float odom_x = 0;
    float odom_y = 0;
    float odom_x_vel = 0;
    float odom_y_vel = 0;

    float offsetX = 0;
    float offsetY = 0;
};

} // namespace communication