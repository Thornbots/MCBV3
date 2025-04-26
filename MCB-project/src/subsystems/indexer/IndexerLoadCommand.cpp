#include "IndexerLoadCommand.hpp"

namespace commands
{

void IndexerLoadCommand::initialize() {
    indexer->resetBallsCounter();
}

void IndexerLoadCommand::execute()
{
    if (indexer->getNumBallsShot() < 0.9) { //make first shot fast, but don't make second fast
        indexer->indexAtMaxRate();
    } else {
        indexer->indexAtRate(ballsPerSecond);
    }
}

void IndexerLoadCommand::end(bool) {
    indexer->stopIndex();
}

bool IndexerLoadCommand::isFinished(void) const {
    if (numBalls < 0) {
        return false;
    }

    return indexer->getNumBallsShot() >= numBalls;
}
}  // namespace commands