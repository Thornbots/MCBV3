#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsObject.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

/* Simple as in not containing anything else. Maybe AtomicGraphicsObject is a better name. */
class SimpleGraphicsObject : public GraphicsObject {
public:
    SimpleGraphicsObject(RefSerialData::Tx::GraphicColor color) : color(color) { UISubsystem::formatGraphicName(graphicNameArray, UISubsystem::getUnusedGraphicName()); }

    int countNeedRedrawn() final { return needsRedrawn(); }

    /*
     * Inheriting simple objects should keep track of what they drew
     * previously with, and compare that to what they want to be drawn with
     * */
    virtual bool needsRedrawn() = 0;

    GraphicsObject* getNext() final {
        if (countIndex == 0 && (isHidden!=wasHidden || needsRedrawn())) {
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
            getNextOperation(),
            0,
            color);
        wasHidden = isHidden;
        finishConfigGraphicData(graphicData);
    }

    void resetIteration() final { countIndex = 0; }

    void hasBeenCleared() final { 
        wasHidden = true; //was deleted, doesn't set if I want to be hidden or not
    }

    RefSerialData::Tx::GraphicColor color;  // can set this directly, will appear next time drawn

    void hide() final {
        isHidden = true;
    }

    void show() final {
        isHidden = false;
    }

private:
    RefSerialData::Tx::GraphicOperation getNextOperation() {
        if(isHidden){
            return RefSerialData::Tx::GraphicOperation::GRAPHIC_DELETE;
        } else {
            if(wasHidden)
                return RefSerialData::Tx::GraphicOperation::GRAPHIC_ADD;
            else
                return RefSerialData::Tx::GraphicOperation::GRAPHIC_MODIFY;
        }
    }

protected:
    bool isHidden = false;
    bool wasHidden = true;

    uint8_t graphicNameArray[3];
};