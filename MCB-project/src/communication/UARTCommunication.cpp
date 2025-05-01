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
        // memset(&lastCVData, 0, sizeof(lastCVData));
        memset(&lastROSData, 0, sizeof(lastROSData));
        // Initial time
        lastReceivedTime = getCurrentTime();
    }

    void UARTCommunication::messageReceiveCallback(const ReceivedSerialMessage &completeMessage)
    {

        size_t bytesToCopy = completeMessage.header.dataLength;
        if (bytesToCopy > sizeof(CVData))
        {
            bytesToCopy = sizeof(CVData);
        }

        if (bytesToCopy > 0 && completeMessage.data != nullptr)
        {
            // memcpy(&lastCVData, completeMessage.data, bytesToCopy);
            memcpy(&lastROSData, completeMessage.data, bytesToCopy);
            if (bytesToCopy < sizeof(CVData))
            {
                // need to do this for pointer arithmetic
                memset(reinterpret_cast<uint8_t*>(&lastROSData) + bytesToCopy, 0, sizeof(CVData) - bytesToCopy);
            }
            // lastCVData.timestamp = getCurrentTime();
            hasNewData = true;
            lastReceivedTime = getCurrentTime();
        }
        else
        {
            hasNewData = false;
        }
    }

    // Will we constantly receive data in a stream?
    void UARTCommunication::update()
    {
        // updateSerial();
        
        if ((getCurrentTime() - lastReceivedTime) > CONNECTION_TIMEOUT)
        {
            hasNewData = false;
        }
    }

    const CVData* UARTCommunication::getLastCVData()
    {
        return hasNewData ? &lastCVData : nullptr;
    }
    
    const ROSData* UARTCommunication::getLastROSData()
    {
        return hasNewData ? &lastROSData : nullptr;
    }

    bool UARTCommunication::sendAutoAimOutput(AutoAimOutput &output)
    {
        // Flexible ports?
        tap::communication::serial::Uart::UartPort currentPort = port;
        // Update the timestamp before sending.
        // output.timestamp = getCurrentTime();
        const uint8_t* msgData = reinterpret_cast<const uint8_t*>(&output);
        int bytesWritten = drivers->uart.write(currentPort, msgData, sizeof(AutoAimOutput));
        return (bytesWritten == sizeof(AutoAimOutput));
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