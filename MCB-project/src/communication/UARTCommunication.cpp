#include "UARTCommunication.hpp"

#include <chrono>
#include <cstring>

namespace communication {
UARTCommunication::UARTCommunication(tap::Drivers* drivers, tap::communication::serial::Uart::UartPort _port, bool isRxCRCEnforcementEnabled)
    : DJISerial(drivers, _port, isRxCRCEnforcementEnabled),
      port(_port),
      hasNewData(false) {
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
    if (completeMessage.header.dataLength <= 0 || completeMessage.data == nullptr) return;
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
    // Flexible ports?
    tap::communication::serial::Uart::UartPort currentPort = port;
    // Update the timestamp before sending.
    // output.timestamp = getCurrentTime();
    if (dataLen > SERIAL_RX_BUFF_SIZE) {
        // RAISE_ERROR(drivers, "received message length longer than allowed max");
        return false;
    }
    outgoingDataFrame msg(dataLen, messageType, dataToBeSent);

    const size_t frameLen = HEADER_SIZE +        // everything before data
                            dataLen +            // payload
                            CRC16_TRAILER_SIZE;  // CRC16

    int bytesWritten = drivers->uart.write(port, reinterpret_cast<uint8_t*>(&msg), frameLen);

    return bytesWritten == static_cast<int>(frameLen);
}

bool UARTCommunication::isConnected() const { return ((getCurrentTime() - lastReceivedTime) <= CONNECTION_TIMEOUT); }

void UARTCommunication::clearNewDataFlag() { hasNewData = false; }

uint64_t UARTCommunication::getCurrentTime() const {
    auto now = std::chrono::system_clock::now();

    // Convert the current time to time since epoch
    auto duration = now.time_since_epoch();

    // Convert duration to milliseconds
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    return milliseconds;
}

tap::communication::serial::Uart::UartPort UARTCommunication::getPort() const { return port; }
}  // namespace communication