#include "UIScheduler.hpp"

namespace ui {

// define the static variable, if this isn't here things go wrong saying it is a undefined reference
uint32_t UIScheduler::currGraphicName = 0;

UIScheduler::UIScheduler(tap::Drivers* drivers) : drivers(drivers), refSerialTransmitter(drivers) { }

uint32_t UIScheduler::getUnusedGraphicName() {
    if (currGraphicName > 0xffffff) {
        // maybe signal some error? turn on the red led?
        // But we would need to draw 16 million things to get here.
        // You would need to draw each pixel on the screen individally, 8 times
        return currGraphicName;
    } else {
        return currGraphicName++;
    }
}

uint8_t* UIScheduler::formatGraphicName(uint8_t array[3], uint32_t name) {
    // shifts and bitwise and
    array[0] = static_cast<uint8_t>((name >> 16) & 0xff);
    array[1] = static_cast<uint8_t>((name >> 8) & 0xff);
    array[2] = static_cast<uint8_t>(name & 0xff);
    return array;
}

// guaranteed to be called, whether we have a command (topLevelContainer) or not
void UIScheduler::refresh() {
    run();
}

bool UIScheduler::run() {
    if (!this->isRunning()) {
        // Need to figure out how often this happens
        restart();  // Restart the thread

        // don't need to reset currGraphicName because old objects don't lose their names they just need to re-draw themselves
        needToDelete = true;
    }

    PT_BEGIN(); //ignore the red underline, it still builds. Not sure how to make it not red underline.
    // inside of a protothread, you aren't able to make new variables

    PT_WAIT_UNTIL(drivers->refSerial.getRefSerialReceivingData());
    
    // need to figure out delete at start. It seems like it wants to delete every time run is called
    if (needToDelete) {
        needToDelete = false;  // probably this needs to be first to not go in this branch next time, so set it to false before the pt call
        PT_CALL(refSerialTransmitter.deleteGraphicLayer(RefSerialTransmitter::Tx::DELETE_ALL, 0));
        topLevelContainer.hasBeenCleared();

        //need to wait for graphics to delete. This might wait longer than is required, but it allows things to draw.
        delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&messageCharacter));
        PT_WAIT_UNTIL(delayTimeout.execute());
    }

    // If we try to restart the hud, break out of the loop
    while (!needToRestart) {
        // graphicsIndex = 0; //might start with one or two already in the array from last time
        hasResetIteration = false;
        nextGraphicsObject = topLevelContainer.getNext();  // if nullptr on first call, this while loop is skipped
        while (nextGraphicsObject) {
            if (nextGraphicsObject->isStringGraphic()) {
                // if it is a string, keep the array as it is and send the string on its own
                nextGraphicsObject->configCharacterData(&messageCharacter);
                PT_CALL(refSerialTransmitter.sendGraphic(&messageCharacter));
                delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&messageCharacter));
                PT_WAIT_UNTIL(delayTimeout.execute());
            } else {
                // if it isn't a string, add it to the array and see if it is full
                objectsToSend[graphicsIndex++] = nextGraphicsObject;
                if (graphicsIndex == TARGET_NUM_OBJECTS) break; //if full, stop trying to find more
            }
            nextGraphicsObject = topLevelContainer.getNext();

            //if we run out of objects, try looping around to the start
            if(!nextGraphicsObject && !hasResetIteration){
                topLevelContainer.resetIteration();

                //don't loop around to the start again
                hasResetIteration = true;
                //if we run out of objects again, we will send however many we got
            }
        }
        

        // drivers->leds.set(tap::gpio::Leds::Red, graphicsIndex == 1);
        // drivers->leds.set(tap::gpio::Leds::Green, graphicsIndex == 7);

        int numToSend = graphicsIndex;
        if(numToSend==3 || numToSend==4){
            //can't send 3, will send 2 and save 1 for later
            //can't send 4, will send 2 and save 2 for later
            numToSend=2;
        }
        if(numToSend==6){
            //can't send 6, will send 5 and save 1 for later
            numToSend=5;
        }

        // so we have up to 7 objects to update. Would do a switch case but might confict with protothread's switch case
        if (numToSend == 1) {
            objectsToSend[0]->configGraphicData(&message1.graphicData);
            PT_CALL(refSerialTransmitter.sendGraphic(&message1));
            delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message1));
        } else if (numToSend == 2) {
            for (innerGraphicsIndex = 0; innerGraphicsIndex < graphicsIndex; innerGraphicsIndex++) objectsToSend[innerGraphicsIndex]->configGraphicData(&message2.graphicData[innerGraphicsIndex]);
            PT_CALL(refSerialTransmitter.sendGraphic(&message2));
            delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message2));
        } else if (numToSend == 5) {
            for (innerGraphicsIndex = 0; innerGraphicsIndex < graphicsIndex; innerGraphicsIndex++) objectsToSend[innerGraphicsIndex]->configGraphicData(&message5.graphicData[innerGraphicsIndex]);
            PT_CALL(refSerialTransmitter.sendGraphic(&message5));
            delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message5));
        } else if (numToSend == 7) {
            for (innerGraphicsIndex = 0; innerGraphicsIndex < graphicsIndex; innerGraphicsIndex++) objectsToSend[innerGraphicsIndex]->configGraphicData(&message7.graphicData[innerGraphicsIndex]);
            PT_CALL(refSerialTransmitter.sendGraphic(&message7));
            delayTimeout.restart(RefSerialData::Tx::getWaitTimeAfterGraphicSendMs(&message7));
        } 
        
        if (graphicsIndex == 4) { //save 2
            objectsToSend[0] = objectsToSend[2]; //copy the third item into the first slot
            objectsToSend[1] = objectsToSend[3]; //copy the fourth item into the second slot
            graphicsIndex = 2; //start with those two next time
        } else if (graphicsIndex == 3 || graphicsIndex == 6) { //save 1
            objectsToSend[0] = objectsToSend[graphicsIndex-1]; //move the 1 we are saving to the front
            graphicsIndex = 1; //start with that one next time
        } else { //don't save any
            graphicsIndex = 0;
        }

        //if we sent something, wait for it so we don't lose packets
        if(numToSend>0)
            PT_WAIT_UNTIL(delayTimeout.execute());

        PT_YIELD();
    }
    // Breaking out of the loop successfully calls this method,
    // allowing us to know that all execution is over.
    PT_END();
}

}  // namespace ui