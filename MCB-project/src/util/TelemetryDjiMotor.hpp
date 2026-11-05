// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "tap/motor/dji_motor.hpp"
#include "drivers.hpp"
#include <utility>

namespace src {
#ifdef MCB_CAN_BUS_HEALTH
class TelemetryDjiMotor : public tap::motor::DjiMotor {
public:
    template <typename... Args>
    explicit TelemetryDjiMotor(src::Drivers* drivers, Args&&... args)
        : tap::motor::DjiMotor(drivers, std::forward<Args>(args)...), health(drivers->canHealth) {}

    void processMessage(const modm::can::Message& message) override {
        health.feedback(getCanBus() == tap::can::CanBus::CAN_BUS1 ? 0 : 1, getMotorIdentifier());
        tap::motor::DjiMotor::processMessage(message);
    }

private:
    CanBusHealth& health;
};
#else
using TelemetryDjiMotor = tap::motor::DjiMotor;
#endif
} // namespace src
