#pragma once

#include "i2c/inc/stm32f4xx_hal.hpp"
#include <cstdint>
#include <cmath>

namespace communication {

class MT6701 {
public:
    void init(I2C_HandleTypeDef* bus) {
        hi2c = bus;
    }

    void startRead() {
        if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) return;

        reg = ANGLE_ADDR;
        HAL_I2C_Master_Transmit_IT(hi2c, ADDRESS << 1, &reg, 1);
        state = State::WAIT_MSB;
    }

    void handleRxComplete(I2C_HandleTypeDef* bus) {
        if (bus != hi2c) return;

        if (state == State::WAIT_MSB) {
            HAL_I2C_Master_Receive_IT(hi2c, ADDRESS << 1, &buffer[0], 1);
            state = State::WAIT_LSB;
        }
        else if (state == State::WAIT_LSB) {
            HAL_I2C_Master_Transmit_IT(hi2c, ADDRESS << 1, &nextReg, 1);
            state = State::WAIT_LSB_2;
        }
        else if (state == State::WAIT_LSB_2) {
            HAL_I2C_Master_Receive_IT(hi2c, ADDRESS << 1, &buffer[1], 1);
            state = State::DONE;
        }
        else if (state == State::DONE) {
            angle = (buffer[0] << 6) | (buffer[1] >> 2);
            state = State::IDLE;
        }
    }

    float getAngle() const {
        return getRawAngle() * 2.0f * 3.1415926f / 16384.0f;
    }

    uint16_t getRawAngle() const {
        return angleInverted ? (16383 - angle) : angle;
    }

    void setAngleInvertedTrue() { angleInverted = true; }
    void setAngleInvertedFalse() { angleInverted = false; }

private:
    enum class State {
        IDLE,
        WAIT_MSB,
        WAIT_LSB,
        WAIT_LSB_2,
        DONE
    };

    I2C_HandleTypeDef* hi2c = nullptr;

    static constexpr uint8_t ADDRESS = 0x06;
    static constexpr uint8_t ANGLE_ADDR = 0x03;

    uint8_t reg;
    uint8_t nextReg = ANGLE_ADDR + 1;

    uint8_t buffer[2]{};
    uint16_t angle = 0;

    bool angleInverted = false;
    State state = State::IDLE;
};

} // namespace communication