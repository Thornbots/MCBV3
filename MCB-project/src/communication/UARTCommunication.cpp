#include "UARTCommunication.hpp"
#include <cstring> 
#include <chrono>

namespace communication
{
    UARTCommunication::UARTCommunication(tap::Drivers *drivers, tap::communication::serial::Uart::UartPort _port, bool isRxCRCEnforcementEnabled) :
        DJISerial(drivers, _port, isRxCRCEnforcementEnabled),
        port(_port),
        hasNewData(false)
    {
        // Good practice
        memset(&mostRecentMessage,0,sizeof(uartMsg));
        // Initial time
        lastReceivedTime = getCurrentTime();
    }

    void UARTCommunication::messageReceiveCallback(const ReceivedSerialMessage &completeMessage)
    {
        if(completeMessage.header.dataLength <= 0 || completeMessage.data  == nullptr)
            return;
        mostRecentMessage.messageType = completeMessage.messageType;
        mostRecentMessage.dataLength = completeMessage.header.dataLength;
        memcpy((void*)mostRecentMessage.data, completeMessage.data, completeMessage.header.dataLength);
        hasNewData = true;
        lastReceivedTime = getCurrentTime();
    }

    // Will we constantly receive data in a stream?
    void UARTCommunication::update()
    {
        updateSerial();
        
        if ((getCurrentTime() - lastReceivedTime) > CONNECTION_TIMEOUT)
        {
            hasNewData = false;
        }
    }

    const UARTCommunication::uartMsg UARTCommunication::getLastMsg()
    {
        return mostRecentMessage;
    }

    bool UARTCommunication::sendMsg(uint8_t *dataToBeSent, uint16_t messageType, uint16_t dataLen)
    {
        // Flexible ports?
        tap::communication::serial::Uart::UartPort currentPort = port;
        // Update the timestamp before sending.
        // output.timestamp = getCurrentTime();
        if (dataLen >= SERIAL_RX_BUFF_SIZE)
        {
            // RAISE_ERROR(drivers, "received message length longer than allowed max");
            return false;
        }


        outgoingDataFrame msg_data(dataLen, messageType, dataToBeSent);
        size_t numBytesToSend =
            sizeof(msg_data.head)+ // head byte (0xA0)
            sizeof(msg_data.dataLen) + // 2 bytes for data length
            sizeof(msg_data.messageType) + // 2 bytes msg type 
            dataLen + // dataToBeSent
            sizeof(uint16_t); // crc

        int bytesWritten = drivers->uart.write(currentPort, (uint8_t*)&msg_data, numBytesToSend);
        return (bytesWritten == sizeof(outgoingDataFrame));
    }

    bool UARTCommunication::isConnected() const
    {
        return ((getCurrentTime() - lastReceivedTime) <= CONNECTION_TIMEOUT);
    }

    void UARTCommunication::clearNewDataFlag()
    {
        hasNewData = false;
    }

    uint64_t UARTCommunication::getCurrentTime() const
    {
        auto now = std::chrono::system_clock::now();

        // Convert the current time to time since epoch
        auto duration = now.time_since_epoch();
    
        // Convert duration to milliseconds
        auto milliseconds
            = std::chrono::duration_cast<std::chrono::milliseconds>(
                  duration)
                  .count();
        return milliseconds;
    }

    tap::communication::serial::Uart::UartPort UARTCommunication::getPort() const
    {
        return port;
    }
}