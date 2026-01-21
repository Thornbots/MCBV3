#pragma once

#include <cstdint>
#include <cstring>

#include "tap/algorithms/crc.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

namespace communication {

class UARTCommunication : public tap::communication::serial::DJISerial {
public:
    struct uartMsg {
        uint16_t messageType;
        uint16_t dataLength;
        uint8_t data[SERIAL_RX_BUFF_SIZE];
    } modm_packed;

    struct outgoingDataFrame {
        uint8_t head;
        uint16_t dataLen;
        uint8_t seqNumber;
        uint8_t crc8;
        uint16_t messageType;
        uint8_t data[SERIAL_RX_BUFF_SIZE + 2];  // +2 for CRC16
        static constexpr size_t HEADER_SIZE = offsetof(outgoingDataFrame, data);

        outgoingDataFrame(uint16_t len, uint16_t msgType, const uint8_t* dataToBeSent) {
            head = SERIAL_HEAD_BYTE;
            dataLen = len;
            messageType = msgType;
            seqNumber = 0;

            std::memcpy(data, dataToBeSent, dataLen);

            // CRC8 covers fixed header fields only
            crc8 = tap::algorithms::calculateCRC8(reinterpret_cast<uint8_t*>(this), offsetof(outgoingDataFrame, crc8));

            // CRC16 covers header + payload
            const size_t crc16Coverage = HEADER_SIZE + dataLen;

            uint16_t crc16 = tap::algorithms::calculateCRC16(reinterpret_cast<uint8_t*>(this), crc16Coverage);

            data[dataLen] = crc16 & 0xFF;
            data[dataLen + 1] = crc16 >> 8;
        }
    } modm_packed;

    UARTCommunication(tap::Drivers* drivers, tap::communication::serial::Uart::UartPort port, bool isRxCRCEnforcementEnabled);

    virtual ~UARTCommunication() = default;

    virtual void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

    void update();

    const uartMsg getLastMsg();

    void clearNewDataFlag();

    bool sendMsg(uint8_t* dataToBeSent, uint16_t messageType, uint16_t dataLen);

    bool isConnected() const;

    bool isFinishedWriting() const { return drivers->uart.isWriteFinished(port); }

    inline bool hasNewMessage() { return hasNewData; }

    uint64_t getCurrentTime() const;

    tap::communication::serial::Uart::UartPort getPort() const;

private:
    bool hasNewData;
    uint64_t lastReceivedTime;

    // TODO: maybe implement cicular queue one day to prevent starving of msg
    //  static constexpr int queueLength = 5;
    //  ReceivedSerialMessage messageQueue[queueLength];
    uartMsg mostRecentMessage;

    const tap::communication::serial::Uart::UartPort port;
    bool rxCRCEnforcementEnabled;

    static constexpr uint32_t CONNECTION_TIMEOUT = 1000;  // Timeout in ms
};

}  // namespace communication
