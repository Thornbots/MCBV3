#pragma once

#include <vector>

#include "util/ui/GraphicsObject.hpp"

class GraphicsContainer : public GraphicsObject {
public:

    GraphicsObject* getNext() final {
        GraphicsObject* r = nullptr;

        // note that it is possible to skip this loop entirely if countIndex==objects.size(),
        // allowing the container above this one to check the container after this one
        // no int i = something, so start with semicolon
        for (; countIndex < objects.size(); countIndex++) {
            if (r) break; //if we have something, don't ask for a new thing
            r = objects.at(countIndex)->getNext();
            if(r) countIndex--; //if the container returned something, it might return more
        }

        // we found something in the loop: return it
        // we didn't find something in the loop: return nullptr, unchanged through the loop
        return r;
    }

    void resetIteration() final {
        countIndex = 0;
        for (GraphicsObject* p : objects) {
            p->resetIteration();
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