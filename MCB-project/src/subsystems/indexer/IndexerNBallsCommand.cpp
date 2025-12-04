#include "IndexerNBallsCommand.hpp"

namespace commands
{

void IndexerNBallsCommand::initialize() {
    // canStart = indexer->heatAllowsShooting() && indexer->isProjectileAtBeam();
    indexer->resetBallsCounter();
    
    // not sure how to do continuous firing with position control yet
    // if(numBalls>=0){
    //     indexer->incrementTargetNumBalls(numBalls);
    //     indexer->setBallsPerSecond(ballsPerSecond);
    // }
    timer.restart(1000/ballsPerSecond);
    
    //for first shot
    indexer->incrementTargetNumBalls(1); 
    numBallsRemaining = numBalls-1; //may end up setting numBallsRemaining to -2, should be fine
}

void IndexerNBallsCommand::execute()
{
    // numBalls was either -1 at the start, or some positive number
    // each time we incrementTargetNumBalls, decrement numBalls, stop when it reaches 0
    if(numBallsRemaining!=0){
        if(indexer->heatAllowsShooting() && timer.execute()){
            indexer->incrementTargetNumBalls(1);
            if(numBallsRemaining>0) numBallsRemaining--;
        }
    }
    
    indexer->indexAtRate(ballsPerSecond);
    
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