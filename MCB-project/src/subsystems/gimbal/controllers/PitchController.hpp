namespace subsystems
{
class PitchController
{
public:
    PitchController();
    //~PitchController();
    float calculate(float currentPosition, float currentVelocity, float targetPosition, float deltaT);

    void clearBuildup() { buildup = 0; };
    
    int signum(float num) { return (num > 0) ? 1 : ((num < 0) ? -1 : 0); }


private:
    // START getters and setters
    float pastTarget = 0;
    float buildup = 0;
    float pastTargetVelo = 0;
    float pastOutput = 0;

};
}  // namespace subsystems
