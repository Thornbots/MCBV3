#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsObject.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class AtomicGraphicsObject : public GraphicsObject {
public:
    AtomicGraphicsObject(RefSerialData::Tx::GraphicColor color) : color(color), graphicName(UISubsystem::getUnusedGraphicName()) {}

    /*
     * Inheriting simple objects should keep track of what they drew
     * previously with, and compare that to what they want to be drawn with
     * */
    virtual bool needsRedrawn() = 0;

    GraphicsObject* getNextBasic() final {
        if(needsSentAsString()) return nullptr;
        if (index_getNextBasic == 0 && !markedToDraw && (isHidden!=wasHidden || layer!=prevLayer || needsRedrawn())) {
            index_getNextBasic = 1;
            return this;
        }
        return nullptr;
    }
    GraphicsObject* getNextBasicRemove() final {
        if(needsSentAsString() || !isRemoving()) return nullptr;
        if (index_getNextBasicRemove == 0 && !markedToDraw && (isHidden!=wasHidden || layer!=prevLayer || needsRedrawn())) {
            index_getNextBasicRemove = 1;
            return this;
        }
        return nullptr;
    }
    GraphicsObject* getNextBasicAdd() final {
        if(needsSentAsString() || !isAdding()) return nullptr;
        if (index_getNextBasicAdd == 0 && !markedToDraw && (isHidden!=wasHidden || layer!=prevLayer || needsRedrawn())) {
            index_getNextBasicAdd = 1;
            return this;
        }
        return nullptr;
    }
    GraphicsObject* getNextText() final {
        if(!needsSentAsString()) return nullptr;
        if (index_getNextText == 0 && !markedToDraw && (isHidden!=wasHidden || layer!=prevLayer || needsRedrawn())) {
            index_getNextText = 1;
            return this;
        }
        return nullptr;
    }

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) = 0;

    void configGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        uint8_t graphicNameArray[3];
        RefSerialTransmitter::configGraphicGenerics(
            graphicData,
            UISubsystem::formatGraphicName(graphicNameArray, graphicName),
            getNextOperation(),
            layer<0 ? 0 : layer, //UISubsystem::getUnusedLayer might return -1 when there aren't any unused layers,
            // if someone doesn't check if it did this protects trying to send -1 to the server
            color);
        wasHidden = isHidden;
        prevLayer = layer;
        finishConfigGraphicData(graphicData);
    }

    void layerHasBeenCleared(int8_t clearedLayer) final {
        if(clearedLayer==layer)
            wasHidden = true; //was deleted, doesn't set if I want to be hidden or not
    }

    void allLayersCleared() final {
        wasHidden = true; //was deleted, doesn't set if I want to be hidden or not
    }

    RefSerialData::Tx::GraphicColor color;  // can set this directly, will appear next time drawn

    void hide() final {
        isHidden = true;
    }

    void show() final {
        isHidden = false;
    }

    void resetDrawMarks() final {
        markedToDraw = false;
    }

    void markToDraw() final {
        markedToDraw = true;
    }
    
    bool isAdding() final {
        return !this->isHidden && this->wasHidden; //set to shown but wasn't previously
    }
    bool isRemoving() final {
        return this->isHidden && !this->wasHidden; //set to hide but wasn't previously
    }
    
    void swapWith(GraphicsObject* o) final {
        
        if((this->isRemoving() && other->isAdding())||
           (this->isAdding() && other->isRemoving())){
            //only AtomicGraphicsObject would ever be removing or adding
            // containers never remove or add, so we can cast here
            AtomicGraphicsObject* other = (AtomicGraphicsObject*) o; 
            
            std::swap(this->graphicName, other->graphicName); //swap id's
            std::swap(this->wasHidden, other->wasHidden); //swap what we think we were
            // so now the one who was removing thinks they are removed
            // so now the one who was adding thinks they are added
            
            // uisubsystem already knows about both of these objects because it called swap
            // so it should know who to send, and they will use GRAPHIC_MODIFY to update
        }
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
    bool markedToDraw = false;

    uint32_t graphicName;
    int8_t layer = 0;

private:
    int8_t prevLayer = -2;
    u_int16_t countIndex = 0;
};