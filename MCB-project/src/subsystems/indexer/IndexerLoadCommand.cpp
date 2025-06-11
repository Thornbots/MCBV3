#include "IndexerLoadCommand.hpp"

namespace commands
{

void IndexerLoadCommand::initialize() {
    indexer->resetBallsCounter();
}

void IndexerLoadCommand::execute()
{
    indexer->loadAtRate(LOAD_BALL_PER_SECOND); // Set the indexer to a not so low speed
}

void IndexerLoadCommand::end(bool) {
    indexer->stopIndex();
}

bool IndexerLoadCommand::isFinished(void) const {
    // Check if the indexer has loaded the balls
    return indexer->isProjectileAtBeam() || !drivers->remote.isConnected();
}
}  // namespace commands