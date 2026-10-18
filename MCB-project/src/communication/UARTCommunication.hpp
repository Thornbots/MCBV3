#pragma once

#include <cstdint>
#include <cstring>
#include <type_traits>
#include "McbDataQueue.hpp"

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
        
        outgoingDataFrame(uint16_t len, uint16_t msgType, const uint8_t* dataToBeSent);
    } modm_packed;
    

    UARTCommunication(tap::Drivers* drivers, tap::communication::serial::Uart::UartPort port, bool isRxCRCEnforcementEnabled);

    virtual ~UARTCommunication() = default;

    virtual void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

    void update();

    const uartMsg getLastMsg();

    void clearNewDataFlag();

    bool sendMsg(uint8_t* dataToBeSent, uint16_t messageType, uint16_t dataLen);

    bool isConnected() const;

    bool isFinishedWriting() const {
        return (!wireStarted || static_cast<int32_t>(getCurrentTime() - wireBusyUntil) >= 0) && drivers->uart.isWriteFinished(port);
    }

    // Copies opaque bytes; false means rejected or queue full. Scheduler context only.
    bool mcbLog(uint8_t channel, const void* data, size_t length) { return dataQueue.enqueue(channel, data, length); }
    template<typename T> bool mcbLog(uint8_t channel, const T& data) {
        static_assert(std::is_trivially_copyable_v<T> && sizeof(T) <= McbDataQueue::MAX_DATA);
        return mcbLog(channel, &data, sizeof(T));
    }
    bool mcbText(const char* format, ...) __attribute__((format(printf, 2, 3)));
    uint32_t droppedData() const { return dataQueue.dropped(); }
    void queuePing(uint8_t number) { pendingPing = number; hasPendingPing = true; }
    void sendIdle(uint32_t telemetryDeadline);

    inline bool hasNewMessage() { return hasNewData; }

    // Milliseconds since boot, wrapping every 2^32 ms.
    uint32_t getCurrentTime() const;

    tap::communication::serial::Uart::UartPort getPort() const;

private:
    McbDataQueue dataQueue;
    uint32_t wireBusyUntil = 0;
    bool wireStarted = false;
    uint8_t pendingPing = 0;
    bool hasPendingPing = false;
    bool hasNewData;
    uint32_t lastReceivedTime;

    // TODO: maybe implement cicular queue one day to prevent starving of msg
    //  static constexpr int queueLength = 5;
    //  ReceivedSerialMessage messageQueue[queueLength];
    uartMsg mostRecentMessage;

    const tap::communication::serial::Uart::UartPort port;

    static constexpr uint32_t CONNECTION_TIMEOUT = 1000;  // Timeout in ms
    
    // for outgoingDataFrame
    static constexpr size_t HEADER_SIZE = offsetof(outgoingDataFrame, data);
    static constexpr size_t CRC8_COVERAGE = offsetof(outgoingDataFrame, crc8);
    static constexpr size_t CRC16_TRAILER_SIZE = sizeof(uint16_t);
};

}  // namespace communication
