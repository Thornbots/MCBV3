#pragma once

#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/algorithms/crc.hpp"
#include "tap/drivers.hpp"
#include <cstdint>
#include <cstring>

namespace communication {

class UARTCommunication : public tap::communication::serial::DJISerial
{
public:
    struct uartMsg{
            uint16_t messageType;
            uint16_t dataLength;
            uint8_t data[SERIAL_RX_BUFF_SIZE];
    } modm_packed;

    struct outgoingDataFrame {
        uint8_t head = SERIAL_HEAD_BYTE;
        uint16_t dataLen;
        uint16_t messageType;
        uint8_t data[SERIAL_RX_BUFF_SIZE];
        outgoingDataFrame(uint16_t dataLen, uint16_t messageType, uint8_t *dataToBeSent): dataLen(dataLen), messageType(messageType){
            memcpy(data, dataToBeSent, dataLen);

            size_t raw_msg_len =
                sizeof(head)+ // head byte (0xA0)
                sizeof(dataLen) + // 2 bytes for data length
                sizeof(messageType) + // 2 bytes msg type 
                dataLen; // dataToBeSent

            uint16_t crc = tap::algorithms::calculateCRC16(
                reinterpret_cast<uint8_t *>(this),
                raw_msg_len);
            data[dataLen] = crc & 0xFF;
            data[dataLen+1] = crc >> 8;
        }
    } modm_packed;

    UARTCommunication(tap::Drivers *drivers,
                        tap::communication::serial::Uart::UartPort port,
                        bool isRxCRCEnforcementEnabled);

    virtual ~UARTCommunication() = default;

    virtual void messageReceiveCallback(const ReceivedSerialMessage &completeMessage) override;

    void update();

    const uartMsg getLastMsg();

    void clearNewDataFlag();

    bool sendMsg(uint8_t *dataToBeSent, uint16_t messageType, uint16_t dataLen);

    bool isConnected() const;

    bool isFinishedWriting() const{
        return drivers->uart.isWriteFinished(port);
    }

    inline bool hasNewMessage() { return hasNewData;}

    uint64_t getCurrentTime() const;
    

    tap::communication::serial::Uart::UartPort getPort() const;

private:
    bool hasNewData;
    uint64_t lastReceivedTime;

    //TODO: maybe implement cicular queue one day to prevent starving of msg
    // static constexpr int queueLength = 5;
    // ReceivedSerialMessage messageQueue[queueLength];
    uartMsg mostRecentMessage;

    const tap::communication::serial::Uart::UartPort port;
    bool rxCRCEnforcementEnabled;

    static constexpr uint32_t CONNECTION_TIMEOUT = 1000; // Timeout in ms
};

} // namespace communication
