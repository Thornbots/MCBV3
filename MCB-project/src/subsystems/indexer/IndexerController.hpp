#include "IndexerControllerConstants.hpp"

namespace subsystems {
using namespace subsystems::indexer;
    
class IndexerController {
public:
        IndexerController();
    float calculate(float currentPosition, float currentVelocity, float targetPosition, float inputVelocity, float deltaT);

    void clearBuildup() {
        buildup = 0;
        pastTargetVelocity = 0;
        pastOutput = 0;
    };


private:
    // START getters and setters
    float buildup = 0;
    float pastTargetVelocity = 0;
    float pastOutput = 0;
    int signum(float num) { return (num > 0) ? 1 : ((num < 0) ? -1 : 0); }

    float decelProfile(float poserror, float thetadot, float thetadotinput);

public:
    // Physical constants
};
}  // namespace subsystems
