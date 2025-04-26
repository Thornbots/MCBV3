
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

namespace src {
    class Drivers : public tap::Drivers {


    public:
        Drivers() : tap::Drivers() {}
        communication::I2CCommunication i2c;

    public:
    };  // class Drivers

}  // namespace src

#endif  // DRIVERS_HPP_
