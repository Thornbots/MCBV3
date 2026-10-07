// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#include "hosted/uart.hpp"
#include <array>
#include <deque>
#include <cerrno>
#include <stdexcept>
#include <unistd.h>
namespace {
int jetsonFd = -1;
std::array<std::deque<uint8_t>, 3> incoming;
}
namespace hosted {
void setJetsonFd(int fd) { jetsonFd = fd; }
void queueUart(tap::communication::serial::Uart::UartPort port, const uint8_t* data, size_t n) {
    if (n == 0) return;
    incoming.at(port).insert(incoming.at(port).end(), data, data + n);
}
}
namespace tap::communication::serial {
bool Uart::read(UartPort port, uint8_t* data) { return read(port, data, 1) == 1; }
size_t Uart::read(UartPort port, uint8_t* data, size_t n) {
    if (port == Uart1) {
        auto count = ::read(jetsonFd, data, n);
        if (count >= 0) return count;
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EIO) return 0;
        throw std::runtime_error("hosted UART read failed");
    }
    auto& queue = incoming.at(port);
    size_t count = 0;
    while (count < n && !queue.empty()) {
        data[count++] = queue.front();
        queue.pop_front();
    }
    return count;
}
size_t Uart::discardReceiveBuffer(UartPort port) {
    size_t n = incoming.at(port).size();
    incoming.at(port).clear();
    return n;
}
bool Uart::write(UartPort port, uint8_t data) { return write(port, &data, 1) == 1; }
size_t Uart::write(UartPort port, const uint8_t* data, size_t n) {
    if (port != Uart1) return n;
    auto count = ::write(jetsonFd, data, n);
    if (count >= 0) return count;
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EIO) return 0;
    throw std::runtime_error("hosted UART write failed");
}
bool Uart::isWriteFinished(UartPort) const { return true; }
void Uart::flushWriteBuffer(UartPort) {}
}
