#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

#include "subsystems/indexer/IndexerSubsystem.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class PredictedRemainingShotsIndicator : public GraphicsContainer {
public:
    PredictedRemainingShotsIndicator(IndexerSubsystem* index) : index(index) {
        
    }

    void update() {
        
    }

private:
    IndexerSubsystem* index;
    
};