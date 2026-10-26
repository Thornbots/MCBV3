#include "UARTCommunication.hpp"

#include "tap/architecture/clock.hpp"
#include <cstring>
#include <algorithm>
#include <cstdarg>
#include <cstdio>

namespace communication {
UARTCommunication::UARTCommunication(tap::Drivers* drivers, tap::communication::serial::Uart::UartPort _port, bool isRxCRCEnforcementEnabled)
    : DJISerial(drivers, _port, isRxCRCEnforcementEnabled),
      hasNewData(false),
      port(_port) {
    // Good practice
    memset(&mostRecentMessage, 0, sizeof(uartMsg));
    // Initial time
    lastReceivedTime = getCurrentTime();
}

UARTCommunication::outgoingDataFrame::outgoingDataFrame(uint16_t len, uint16_t msgType, const uint8_t* dataToBeSent) {
    head = SERIAL_HEAD_BYTE;
    dataLen = len;
    messageType = msgType;
    seqNumber = 0;

    std::memcpy(data, dataToBeSent, dataLen);

    // CRC8 covers fixed header fields only
    crc8 = tap::algorithms::calculateCRC8(reinterpret_cast<uint8_t*>(this), CRC8_COVERAGE);

    // CRC16 covers header + payload
    const size_t crc16Coverage = HEADER_SIZE + dataLen;

    uint16_t crc16 = tap::algorithms::calculateCRC16(reinterpret_cast<uint8_t*>(this), crc16Coverage);

    data[dataLen] = crc16 & 0xFF;
    data[dataLen + 1] = crc16 >> 8;
}

void UARTCommunication::messageReceiveCallback(const ReceivedSerialMessage& completeMessage) {
    if (completeMessage.header.dataLength == 0 || completeMessage.header.dataLength > SERIAL_RX_BUFF_SIZE) return;
    mostRecentMessage.messageType = completeMessage.messageType;
    mostRecentMessage.dataLength = completeMessage.header.dataLength;
    memcpy((void*)mostRecentMessage.data, completeMessage.data, completeMessage.header.dataLength);
    hasNewData = true;
    lastReceivedTime = getCurrentTime();
}

// Will we constantly receive data in a stream?
void UARTCommunication::update() {
    updateSerial();

    if ((getCurrentTime() - lastReceivedTime) > CONNECTION_TIMEOUT) {
        hasNewData = false;
    }
}

const UARTCommunication::uartMsg UARTCommunication::getLastMsg() { return mostRecentMessage; }

bool UARTCommunication::sendMsg(uint8_t* dataToBeSent, uint16_t messageType, uint16_t dataLen) {
    // Update the timestamp before sending.
    // output.timestamp = getCurrentTime();
    if (!isFinishedWriting() || dataLen > SERIAL_RX_BUFF_SIZE) {
        // RAISE_ERROR(drivers, "received message length longer than allowed max");
        return false;
    }
    outgoingDataFrame msg(dataLen, messageType, dataToBeSent);

    const size_t frameLen = HEADER_SIZE +        // everything before data
                            dataLen +            // payload
                            CRC16_TRAILER_SIZE;  // CRC16

    int bytesWritten = drivers->uart.write(port, reinterpret_cast<uint8_t*>(&msg), frameLen);

    if (bytesWritten > 0) {
        wireStarted = true;
        wireBusyUntil = getCurrentTime() + McbDataQueue::wireMilliseconds(bytesWritten);
    }
    return bytesWritten == static_cast<int>(frameLen);
}

bool UARTCommunication::mcbText(const char* format, ...) {
    char text[McbDataQueue::MAX_DATA + 1];
    va_list args;
    va_start(args, format);
    const int length = std::vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (length < 0) return false;
    return mcbLog(McbDataQueue::TEXT_CHANNEL, text, std::min<size_t>(length, McbDataQueue::MAX_DATA));
}

void UARTCommunication::sendIdle(uint32_t telemetryDeadline) {
    if (!isFinishedWriting()) return;
    const uint32_t now = getCurrentTime();
    // Echoes use the same deadline guard as data; a new PING replaces a pending echo.
    if (hasPendingPing && McbDataQueue::fitsBefore(now, telemetryDeadline, 10)) {
        if (sendMsg(&pendingPing, 5, 1)) hasPendingPing = false;
        return;
    }
    McbDataQueue::Packet packet;
    if (dataQueue.peek(packet) && McbDataQueue::fitsBefore(now, telemetryDeadline, packet.size + 9)) {
        if (sendMsg(packet.bytes.data(), 6, packet.size)) dataQueue.consume(packet);
    }
}

bool UARTCommunication::isConnected() const { return ((getCurrentTime() - lastReceivedTime) <= CONNECTION_TIMEOUT); }

void UARTCommunication::clearNewDataFlag() { hasNewData = false; }

uint32_t UARTCommunication::getCurrentTime() const {
    return tap::arch::clock::getTimeMilliseconds();
}

tap::communication::serial::Uart::UartPort UARTCommunication::getPort() const { return port; }
}  // namespace communication