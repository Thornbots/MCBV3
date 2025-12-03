#include "IndexerNBallsCommand.hpp"

namespace commands
{

void IndexerNBallsCommand::initialize() {
    indexer->resetBallsCounter();
    
    // not sure how to do continuous firing with position control yet
    if(numBalls>=0){
        indexer->incrementTargetNumBalls(numBalls);
        indexer->setBallsPerSecond(ballsPerSecond);
    }
}
void IndexerNBallsCommand::execute()
{
    indexer->indexAtRate(ballsPerSecond); //counter 
    
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

    return indexer->getNumBallsShot() >= numBalls;
}
}  // namespace commands