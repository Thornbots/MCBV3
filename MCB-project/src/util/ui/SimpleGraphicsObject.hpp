#pragma once

#include "UIScheduler.hpp"
#include "GraphicsObject.hpp"


namespace ui {
using namespace tap::communication::serial;

/* Simple as in not containing anything else. Maybe AtomicGraphicsObject is a better name. */
class SimpleGraphicsObject : public GraphicsObject {
public:
    SimpleGraphicsObject(RefSerialData::Tx::GraphicColor color) : color(color) { UIScheduler::formatGraphicName(graphicNameArray, UIScheduler::getUnusedGraphicName()); }

    int countNeedRedrawn() final { return needsRedrawn(); }

    /*
     * Inheriting simple objects should keep track of what they drew
     * previously with, and compare that to what they want to be drawn with
     * */
    virtual bool needsRedrawn() = 0;

    GraphicsObject* getNext() final {
        if (countIndex == 0 && needsRedrawn()) {  // could do !countIndex
            countIndex = 1;
            return this;
        }
        return nullptr;
    }

    int size() final {
        return 1;  // container of one object
    }

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) = 0;

    void configGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        RefSerialTransmitter::configGraphicGenerics(
            graphicData,
            graphicNameArray,
            hasDrawn ? RefSerialData::Tx::GraphicOperation::GRAPHIC_MODIFY : RefSerialData::Tx::GraphicOperation::GRAPHIC_ADD,
            0,
            color);
        hasDrawn = true;
        finishConfigGraphicData(graphicData);
    }

    void resetIteration() final { countIndex = 0; }

    void hasBeenCleared() final { hasDrawn = false; }

    RefSerialData::Tx::GraphicColor color;  // can set this directly, will appear next time drawn

protected:
    bool hasDrawn = false;  // to determine if we need to modify or add

    uint8_t graphicNameArray[3];
};
}