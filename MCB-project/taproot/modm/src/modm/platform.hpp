/*
 * Copyright (c) 2016, Sascha Schade
 * Copyright (c) 2017, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <modm/architecture.hpp>

#include "platform/adc/adc_1.hpp"
#include "platform/adc/adc_3.hpp"
#include "platform/adc/adc_interrupt_1.hpp"
#include "platform/adc/adc_interrupt_3.hpp"
#include "platform/can/can_1.hpp"
#include "platform/can/can_2.hpp"
#include "platform/can/can_bit_timings.hpp"
#include "platform/can/can_filter.hpp"
#include "platform/clock/rcc.hpp"
#include "platform/clock/systick_timer.hpp"
#include "platform/core/delay_ns.hpp"
#include "platform/core/hardware_init.hpp"
#include "platform/core/heap_table.hpp"
#include "platform/core/peripherals.hpp"
#include "platform/flash/flash.hpp"
#include "platform/gpio/base.hpp"
#include "platform/gpio/connector.hpp"
#include "platform/gpio/connector_detail.hpp"
#include "platform/gpio/gpio_A0.hpp"
#include "platform/gpio/gpio_A1.hpp"
#include "platform/gpio/gpio_A10.hpp"
#include "platform/gpio/gpio_A11.hpp"
#include "platform/gpio/gpio_A12.hpp"
#include "platform/gpio/gpio_A13.hpp"
#include "platform/gpio/gpio_A14.hpp"
#include "platform/gpio/gpio_A15.hpp"
#include "platform/gpio/gpio_A2.hpp"
#include "platform/gpio/gpio_A3.hpp"
#include "platform/gpio/gpio_A4.hpp"
#include "platform/gpio/gpio_A5.hpp"
#include "platform/gpio/gpio_A6.hpp"
#include "platform/gpio/gpio_A7.hpp"
#include "platform/gpio/gpio_A8.hpp"
#include "platform/gpio/gpio_A9.hpp"
#include "platform/gpio/gpio_B0.hpp"
#include "platform/gpio/gpio_B1.hpp"
#include "platform/gpio/gpio_B10.hpp"
#include "platform/gpio/gpio_B11.hpp"
#include "platform/gpio/gpio_B12.hpp"
#include "platform/gpio/gpio_B13.hpp"
#include "platform/gpio/gpio_B14.hpp"
#include "platform/gpio/gpio_B15.hpp"
#include "platform/gpio/gpio_B2.hpp"
#include "platform/gpio/gpio_B3.hpp"
#include "platform/gpio/gpio_B4.hpp"
#include "platform/gpio/gpio_B5.hpp"
#include "platform/gpio/gpio_B6.hpp"
#include "platform/gpio/gpio_B7.hpp"
#include "platform/gpio/gpio_B8.hpp"
#include "platform/gpio/gpio_B9.hpp"
#include "platform/gpio/gpio_C0.hpp"
#include "platform/gpio/gpio_C1.hpp"
#include "platform/gpio/gpio_C10.hpp"
#include "platform/gpio/gpio_C11.hpp"
#include "platform/gpio/gpio_C12.hpp"
#include "platform/gpio/gpio_C13.hpp"
#include "platform/gpio/gpio_C14.hpp"
#include "platform/gpio/gpio_C15.hpp"
#include "platform/gpio/gpio_C2.hpp"
#include "platform/gpio/gpio_C3.hpp"
#include "platform/gpio/gpio_C4.hpp"
#include "platform/gpio/gpio_C5.hpp"
#include "platform/gpio/gpio_C6.hpp"
#include "platform/gpio/gpio_C7.hpp"
#include "platform/gpio/gpio_C8.hpp"
#include "platform/gpio/gpio_C9.hpp"
#include "platform/gpio/gpio_D0.hpp"
#include "platform/gpio/gpio_D1.hpp"
#include "platform/gpio/gpio_D10.hpp"
#include "platform/gpio/gpio_D11.hpp"
#include "platform/gpio/gpio_D12.hpp"
#include "platform/gpio/gpio_D13.hpp"
#include "platform/gpio/gpio_D14.hpp"
#include "platform/gpio/gpio_D15.hpp"
#include "platform/gpio/gpio_D2.hpp"
#include "platform/gpio/gpio_D3.hpp"
#include "platform/gpio/gpio_D4.hpp"
#include "platform/gpio/gpio_D5.hpp"
#include "platform/gpio/gpio_D6.hpp"
#include "platform/gpio/gpio_D7.hpp"
#include "platform/gpio/gpio_D8.hpp"
#include "platform/gpio/gpio_D9.hpp"
#include "platform/gpio/gpio_E0.hpp"
#include "platform/gpio/gpio_E1.hpp"
#include "platform/gpio/gpio_E10.hpp"
#include "platform/gpio/gpio_E11.hpp"
#include "platform/gpio/gpio_E12.hpp"
#include "platform/gpio/gpio_E13.hpp"
#include "platform/gpio/gpio_E14.hpp"
#include "platform/gpio/gpio_E15.hpp"
#include "platform/gpio/gpio_E2.hpp"
#include "platform/gpio/gpio_E3.hpp"
#include "platform/gpio/gpio_E4.hpp"
#include "platform/gpio/gpio_E5.hpp"
#include "platform/gpio/gpio_E6.hpp"
#include "platform/gpio/gpio_E7.hpp"
#include "platform/gpio/gpio_E8.hpp"
#include "platform/gpio/gpio_E9.hpp"
#include "platform/gpio/gpio_F0.hpp"
#include "platform/gpio/gpio_F1.hpp"
#include "platform/gpio/gpio_F10.hpp"
#include "platform/gpio/gpio_F11.hpp"
#include "platform/gpio/gpio_F12.hpp"
#include "platform/gpio/gpio_F13.hpp"
#include "platform/gpio/gpio_F14.hpp"
#include "platform/gpio/gpio_F15.hpp"
#include "platform/gpio/gpio_F2.hpp"
#include "platform/gpio/gpio_F3.hpp"
#include "platform/gpio/gpio_F4.hpp"
#include "platform/gpio/gpio_F5.hpp"
#include "platform/gpio/gpio_F6.hpp"
#include "platform/gpio/gpio_F7.hpp"
#include "platform/gpio/gpio_F8.hpp"
#include "platform/gpio/gpio_F9.hpp"
#include "platform/gpio/gpio_G0.hpp"
#include "platform/gpio/gpio_G1.hpp"
#include "platform/gpio/gpio_G10.hpp"
#include "platform/gpio/gpio_G11.hpp"
#include "platform/gpio/gpio_G12.hpp"
#include "platform/gpio/gpio_G13.hpp"
#include "platform/gpio/gpio_G14.hpp"
#include "platform/gpio/gpio_G15.hpp"
#include "platform/gpio/gpio_G2.hpp"
#include "platform/gpio/gpio_G3.hpp"
#include "platform/gpio/gpio_G4.hpp"
#include "platform/gpio/gpio_G5.hpp"
#include "platform/gpio/gpio_G6.hpp"
#include "platform/gpio/gpio_G7.hpp"
#include "platform/gpio/gpio_G8.hpp"
#include "platform/gpio/gpio_G9.hpp"
#include "platform/gpio/gpio_H0.hpp"
#include "platform/gpio/gpio_H1.hpp"
#include "platform/gpio/gpio_H10.hpp"
#include "platform/gpio/gpio_H11.hpp"
#include "platform/gpio/gpio_H12.hpp"
#include "platform/gpio/gpio_H13.hpp"
#include "platform/gpio/gpio_H14.hpp"
#include "platform/gpio/gpio_H15.hpp"
#include "platform/gpio/gpio_H2.hpp"
#include "platform/gpio/gpio_H3.hpp"
#include "platform/gpio/gpio_H4.hpp"
#include "platform/gpio/gpio_H5.hpp"
#include "platform/gpio/gpio_H6.hpp"
#include "platform/gpio/gpio_H7.hpp"
#include "platform/gpio/gpio_H8.hpp"
#include "platform/gpio/gpio_H9.hpp"
#include "platform/gpio/gpio_I0.hpp"
#include "platform/gpio/gpio_I1.hpp"
#include "platform/gpio/gpio_I10.hpp"
#include "platform/gpio/gpio_I11.hpp"
#include "platform/gpio/gpio_I2.hpp"
#include "platform/gpio/gpio_I3.hpp"
#include "platform/gpio/gpio_I4.hpp"
#include "platform/gpio/gpio_I5.hpp"
#include "platform/gpio/gpio_I6.hpp"
#include "platform/gpio/gpio_I7.hpp"
#include "platform/gpio/gpio_I8.hpp"
#include "platform/gpio/gpio_I9.hpp"
#include "platform/gpio/inverted.hpp"
#include "platform/gpio/port.hpp"
#include "platform/gpio/set.hpp"
#include "platform/gpio/software_port.hpp"
#include "platform/gpio/unused.hpp"
#include "platform/i2c/i2c_master_2.hpp"
#include "platform/random/random_number_generator.hpp"
#include "platform/spi/spi_base.hpp"
#include "platform/spi/spi_hal_1.hpp"
#include "platform/spi/spi_master_1.hpp"
#include "platform/timer/advanced_base.hpp"
#include "platform/timer/basic_base.hpp"
#include "platform/timer/general_purpose_base.hpp"
#include "platform/timer/timer_1.hpp"
#include "platform/timer/timer_10.hpp"
#include "platform/timer/timer_4.hpp"
#include "platform/timer/timer_8.hpp"
#include "platform/uart/uart_1.hpp"
#include "platform/uart/uart_3.hpp"
#include "platform/uart/uart_6.hpp"
#include "platform/uart/uart_base.hpp"
#include "platform/uart/uart_hal_1.hpp"
#include "platform/uart/uart_hal_3.hpp"
#include "platform/uart/uart_hal_6.hpp"
