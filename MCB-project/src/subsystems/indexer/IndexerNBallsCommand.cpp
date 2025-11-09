#include "IndexerNBallsCommand.hpp"

namespace commands
{

void IndexerNBallsCommand::initialize() {
    indexer->resetBallsCounter();
    
    // not sure how to do continuous firing with position control yet
    if(numBalls>=0){
        indexer->incrementTargetNumBalls(numBalls);
    }
}
void IndexerNBallsCommand::execute()
{
    // if (indexer->getNumBallsShot() < INITIAL_BURST_NUM_BALLS) { //make first shot fast, but don't make second fast
    //     indexer->indexAtMaxRate();
    // } else {
    //     indexer->indexAtRate(ballsPerSecond);
    // }
    
    // incrementTargetNumBalls handled it in initialize
}

void IndexerNBallsCommand::end(bool) {
    indexer->stopIndex();
}

bool IndexerNBallsCommand::isFinished(void) const {
    if(!drivers->remote.isConnected()) return true;

    if (numBalls < 0) {
        return false;
    }

    // might need changed with position control
    return indexer->getNumBallsShot() >= numBalls;
}
}  // namespace commands