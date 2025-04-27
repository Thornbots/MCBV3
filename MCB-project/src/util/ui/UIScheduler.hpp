#pragma once
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "modm/processing/protothread/protothread.hpp"

#include "drivers.hpp"
#include "GraphicsContainer.hpp"

namespace ui{

using namespace tap::communication::serial;

class UIScheduler : private ::modm::pt::Protothread
{

public:
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
    bool needToDelete = true; 
    bool hasResetIteration = false;
    static constexpr int TARGET_NUM_OBJECTS = 7; //could change this to be less than 7 for testing, but don't make this larger than 7
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
    GraphicsContainer topLevelContainer;

public:  // Public Methods
    UIScheduler(tap::Drivers* driver);
    ~UIScheduler() {}  // Intentionally blank

    /*
     * Call this function repeatedly, inside of the main loop.
     */
    void refresh();


    /*
     * Gets the integer graphic name, incrementing it for the next time this gets called.
     * Static so that SimpleGraphicsObjects can call this without having to have drivers
     */
    static uint32_t getUnusedGraphicName();

    /*
     * Puts name into array (changing it in place), and returns array.
     * Static so that SimpleGraphicsObjects can call this without having to have drivers
     */
    static uint8_t* formatGraphicName(uint8_t array[3], uint32_t name);


    
private:  // Private Methods
    bool run(); //for protothread

};
}  // namespace ui