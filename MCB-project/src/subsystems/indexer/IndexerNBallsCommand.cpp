#include "IndexerNBallsCommand.hpp"

namespace commands
{

void IndexerNBallsCommand::initialize() {
    target = indexer->getTotalNumBallsShot()+numBalls;
    if(numBalls<0){
        indexer->indexAtRate(ballsPerSecond);
    } else {
        indexer->indexNShotsAtRate(ballsPerSecond, numBalls);
    }
}

void IndexerNBallsCommand::execute()
{
}

void IndexerNBallsCommand::end(bool) {
    indexer->stopIndexingAtRate();
}

bool IndexerNBallsCommand::isFinished(void) const {
    if(!drivers->remote.isConnected() || !indexer->refPoweringIndex()) return true;

    return target == indexer->getTotalNumBallsShot();
}
}  // namespace commands