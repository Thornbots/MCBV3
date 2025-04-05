#pragma once

#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/uart.hpp"
#include "drivers.hpp"
#include <cstdint>
#include <cstring>

namespace communication
{

// Incoming
struct CVData 
{
    float x = 0;          // meters
    float y = 0;          // meters
    float z = 0;          // meters
    float v_x = 0;        // m/s
    float v_y = 0;        // m/s
    float v_z = 0;        // m/s
    float a_x = 0;        // m/s^2
    float a_y = 0;        // m/s^2
    float a_z = 0;        // m/s^2
    float confidence = 0; // 0.0 to 1.0
    // uint64_t timestamp = 0;
};

// Output
struct AutoAimOutput
{
    uint8_t header = 0xA5;           
    uint16_t data_len = sizeof(float); // litle endian
    float pitch;                     
    uint8_t newline = 0x0A;          
    uint64_t timestamp = 0;         
} modm_packed;

class JetsonCommunication : public tap::communication::serial::DJISerial
{
public:
    JetsonCommunication(tap::Drivers *drivers,
                        tap::communication::serial::Uart::UartPort port,
                        bool isRxCRCEnforcementEnabled);

    virtual ~JetsonCommunication() = default;

    virtual void messageReceiveCallback(const ReceivedSerialMessage &completeMessage) override;

    void update();

    const CVData* getLastCVData();

    void clearNewDataFlag();

    bool sendAutoAimOutput(AutoAimOutput &output);

    bool isConnected() const;

    inline bool hasNewMessage() { return hasNewData;}

    uint64_t getCurrentTime() const;

    tap::communication::serial::Uart::UartPort getPort() const;

private:
    CVData lastCVData;
    bool hasNewData;
    uint64_t lastReceivedTime;

    const tap::communication::serial::Uart::UartPort port;
    bool rxCRCEnforcementEnabled;

    static constexpr uint32_t CONNECTION_TIMEOUT = 1000; // Timeout in ms
};

} // namespace communication
