#include "IndexerLoadCommand.hpp"

namespace commands
{

void IndexerLoadCommand::initialize() {
}

void IndexerLoadCommand::execute()
{
    drivers->leds.set(tap::gpio::Leds::Blue, true);
    
    indexer->loadAtRate(LOAD_BALL_PER_SECOND); // Set the indexer to a not so low speed
}

void IndexerLoadCommand::end(bool) {
    drivers->leds.set(tap::gpio::Leds::Blue, false);
    
    indexer->loadAtRate(0);
}

bool IndexerLoadCommand::isFinished(void) const {
    // Check if the indexer has loaded the balls
    return indexer->isProjectileAtBeam() || !drivers->remote.isConnected() || !indexer->refPoweringIndex();
}
}  // namespace commands