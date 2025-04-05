

namespace subsystems
{
class OdometryController
{
public:
    float *torqueHistory; 
    OdometryController();
    //~OdometryController();
    void estimateState(float *theta, float *thetadot, float tLast, float drivetrainVelocity);
    float calculate(float currentPosition, float currentVelocity, float currentDrivetrainVelocity, float targetPosition, float inputVelocity, float deltaT);

    void clearBuildup() { buildup = 0; };

private:
    
    // START getters and setters
    float buildup = 0;
    float pastTargetVelocity = 0;
    float pastOutput = 0;
    float pastTorque = 0;
    int signum(float num) { return (num > 0) ? 1 : ((num < 0) ? -1 : 0); }

    float decelProfile(float poserror, float thetadot, float thetadotinput, float drivetrainVelocity);

public:
    // Physical constants
 };
}  // namespace subsystems
