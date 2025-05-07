#ifdef HERO
#include "IndexerLoadCommand.hpp"

namespace commands
{

void IndexerLoadCommand::initialize() {
    indexer->resetBallsCounter();
}

void IndexerLoadCommand::execute()
{
    indexer->indexAtRate(1.0f); // Set the indexer to a low speed
}

void IndexerLoadCommand::end(bool) {
    indexer->stopIndex();
}

bool IndexerLoadCommand::isFinished(void) const {
    // Check if the indexer has loaded the balls
    return !indexer->readBreakBeam() || !drivers->remote.isConnected(); // Assuming 5 is the number of balls to load
}
}  // namespace commands
#endif