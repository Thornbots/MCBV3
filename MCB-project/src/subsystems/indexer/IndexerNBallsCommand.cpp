#include "IndexerNBallsCommand.hpp"

namespace commands
{

void IndexerNBallsCommand::initialize() {
    indexer->resetBallsCounter();
}
void IndexerNBallsCommand::execute()
{
    if (indexer->getNumBallsShot() < INITIAL_BURST_NUM_BALLS) { //make first shot fast, but don't make second fast
        indexer->indexAtMaxRate();
    } else {
        indexer->indexAtRate(ballsPerSecond);
    }
}

void IndexerNBallsCommand::end(bool) {
    indexer->stopIndex();
}

bool IndexerNBallsCommand::isFinished(void) const {
    if (numBalls < 0) {
        return false;
    }

    return indexer->getNumBallsShot() >= numBalls;
}
}  // namespace commands