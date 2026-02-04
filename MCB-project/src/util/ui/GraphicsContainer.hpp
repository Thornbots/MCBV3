#pragma once

#include <vector>

#include "util/ui/GraphicsObject.hpp"


// name is expected to be one of:
// getNextBasic
// getNextBasicRemove
// getNextBasicAdd
// getNextText
// note that it is possible for the loop to not execute any times if countIndex==objects.size(), allowing the container above this one to check the container after this one
#define GET_NEXT_GENERIC(name)  \
        GraphicsObject* getNext##name() final { \
            GraphicsObject* r = nullptr; \
            for (; index_getNext##name < objects.size(); index_getNext##name ++) { \
                if (r) break; \
                r = objects.at(index_getNext##name)->getNext##name(); \
                if(r) index_getNext##name--; \
            } \
            return r; \
        }

class GraphicsContainer : public GraphicsObject {
public:

    GET_NEXT_GENERIC(Basic)
    GET_NEXT_GENERIC(BasicRemove)
    GET_NEXT_GENERIC(BasicAdd)
    GET_NEXT_GENERIC(Text)

    void resetTextIteration() final {
        GraphicsObject::resetTextIteration();
        for (GraphicsObject* p : objects) {
            p->resetTextIteration();
        }
    }
    void resetBasicIteration() final {
        GraphicsObject::resetBasicIteration();
        for (GraphicsObject* p : objects) {
            p->resetBasicIteration();
        }
    }

    /* When adding, make sure you don't lose the object from leaving scope */
    void addGraphicsObject(GraphicsObject* obj) { objects.push_back(obj); }

    void layerHasBeenCleared(int8_t layer) final {
        for (GraphicsObject* p : objects) {
            p->layerHasBeenCleared(layer);
        }
    }

    void allLayersCleared() final {
        for (GraphicsObject* p : objects) {
            p->allLayersCleared();
        }
    }

    void hide() final {
        for (GraphicsObject* p : objects) {
            p->hide();
        }
    }

    void show() final {
        for (GraphicsObject* p : objects) {
            p->show();
        }
    }

    void resetDrawMarks() final {
        for (GraphicsObject* p : objects) {
            p->resetDrawMarks();
        }
    }


private:
    std::vector<GraphicsObject*> objects;
};