#pragma once

#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "drivers.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "modm/processing/protothread/protothread.hpp"

namespace subsystems
{
using namespace tap::communication::serial;

class UISubsystem : public tap::control::Subsystem, ::modm::pt::Protothread
{
    
public:
    using Color = RefSerialData::Tx::GraphicColor; //makes it so you can use UISubsystem::Color::CYAN
    static constexpr uint16_t SCREEN_WIDTH = 1920; //pixels. x=0 is left
    static constexpr uint16_t SCREEN_HEIGHT = 1080; //pixels. y=0 is bottom
    static constexpr uint16_t HALF_SCREEN_WIDTH = SCREEN_WIDTH/2; //pixels
    static constexpr uint16_t HALF_SCREEN_HEIGHT = SCREEN_HEIGHT/2; //pixels

private:  // Private Variables
    tap::Drivers* drivers;
    RefSerialTransmitter refSerialTransmitter;
    tap::arch::MilliTimeout delayTimeout; //for not sending things too fast and dropping packets

    static uint32_t currGraphicName;
    
    //for protothread
    bool needToRestart = true; 
    int timesResetIteration = 0;
    static constexpr int TARGET_NUM_OBJECTS = 7; //could change this to test if 7 is the most efficient, and test wasteIsBetterForX's, but don't make this larger than 7
    GraphicsObject* objectsToSend[TARGET_NUM_OBJECTS];
    int graphicsIndex=0;
    int innerGraphicsIndex=0;
    int numToSend=0;
    GraphicsObject* nextGraphicsObject=nullptr;
    RefSerialData::Tx::Graphic1Message message1;
    RefSerialData::Tx::Graphic2Message message2;
    RefSerialData::Tx::Graphic5Message message5;
    RefSerialData::Tx::Graphic7Message message7;
    RefSerialData::Tx::GraphicCharacterMessage messageCharacter;

    //when get UIDrawCommand, it should set this
    GraphicsContainer* topLevelContainer = nullptr;

public:  // Public Methods
    UISubsystem(tap::Drivers* driver);
    ~UISubsystem() {}  // Intentionally blank


    /*
     * Call this function once, outside of the main loop.
     */
    void initialize();

    void refresh() override;

    static uint32_t getUnusedGraphicName();

    /*
     * Puts name into array (changing it in place), and returns array
     */
    static uint8_t* formatGraphicName(uint8_t array[3], uint32_t name);

    void setTopLevelContainer(GraphicsContainer* container);

    
private:  // Private Methods
    bool run(); //for protothread


};
}  // namespace subsystems