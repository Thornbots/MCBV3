// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cstddef>
#include <cstdint>
#include "tap/communication/serial/uart.hpp"
namespace hosted {
void queueUart(tap::communication::serial::Uart::UartPort, const uint8_t*, size_t);
void setJetsonFd(int fd);
}
