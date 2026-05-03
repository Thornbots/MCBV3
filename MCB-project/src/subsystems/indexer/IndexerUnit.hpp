#pragma once
#include "IndexerSubsystemConstants.hpp"

#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/motor/dji_motor.hpp"

#include "drivers.hpp"
#include "IndexerController.hpp"


// not too smart
// doesn't know if it is homing
// just gets told to do position control or velocity control to move a single motor
// can also be told to do auto unjam using IndexerSubsystemConstants
namespace subsystems{

class IndexerUnit
{

public:

    IndexerUnit(src::Drivers* drivers, tap::motor::DjiMotor* index, float shaftRevPerBall, float motorRevPerBall) : drivers(drivers), index(index), shaftRevPerBall(shaftRevPerBall), motorRevPerBall(motorRevPerBall)  {
        
    }

    void initialize() {
        index->initialize();
    }

    // uses stored target position to do position control
    void positionControl(){
        index->setDesiredOutput(getIndexerVoltage(index->getPositionUnwrapped()/GEAR_RATIO, getCurrentOutputVelo(), targetIndexerPosition, 0, DT));
    }

    // uses given balls per second and rev per ball to do velo control
    void velocityControl(float ballsPerSecond){
        index->setDesiredOutput(getIndexerVoltage(0, getCurrentOutputVelo(), 0, ballsPerSecond/shaftRevPerBall, DT)); //by giving it 0 target position and velo we effectively have a velo controller
    }
    
    // uses a plain pid controller instead of a more fancy one
    void oldVelocityControl(float ballsPerSecond) {
        indexPIDController.runControllerDerivateError(ballsPerSecond * 60.0f * motorRevPerBall - index->getShaftRPM(), 1);

        index->setDesiredOutput(static_cast<int32_t>(indexPIDController.getOutput()));
    }
    
    void shootOnce() {
        targetIndexerPosition+=getPositionIncrement(); 
    }
    
    
    void clearBuildup() {
        indexerController.clearBuildup();
    }
    
    void doAfterHomingOffset() {
        index->resetEncoderValue();
        targetIndexerPosition = getPositionIncrement()*INITIAL_INDEX_OFFSET;
    }
    
    void doGiveUpHomingOffset() {
        targetIndexerPosition = index->getPositionUnwrapped()/GEAR_RATIO;
    }
    
    // goes only forwards, so don't call repeatedly
    void indexNearest() {        
        float currentPos = index->getPositionUnwrapped()/GEAR_RATIO;  //radians
        //index to the nearest next shot. This ensures we always have a shot ready to shoot
        targetIndexerPosition = (std::ceil((currentPos - getPositionIncrement()*INITIAL_INDEX_OFFSET) //find number of shots we are at
        /getPositionIncrement())
        + INITIAL_INDEX_OFFSET) //add the offset back
        * getPositionIncrement(); //convert to radians
    }

    tap::motor::DjiMotor* index;
    
private:
    src::Drivers* drivers;
    
    // revolutions of OUTPUT SHAFT per ball. Different than the ones per robot in IndexerSubsystemConstants, because those are motor shaft (before the gear box)
    float shaftRevPerBall;

    // same as the one in IndexerSubsystemConstants
    float motorRevPerBall;
    
    tap::algorithms::SmoothPid indexPIDController{PID_CONF_INDEX};

    
    float targetIndexerPosition = 0;
        
    IndexerController indexerController;
    
    
    tap::arch::MilliTimeout timeoutUnjam;
    bool isAutoUnjamming = false;
    
    
    
    float getCurrentOutputVelo() {
        return index->getShaftRPM()*(PI/30)/GEAR_RATIO;
    }

    float getPositionIncrement(){
        return 2*PI*shaftRevPerBall;
    }
    
    
    // position and velo in radians of the output (where spindex is attached) shaft
    int getIndexerVoltage(float currentPosition, float currentVelocity, float targetPosition, float inputVelocity, float deltaT) {
        return 1000 * indexerController.calculate(currentPosition, currentVelocity, targetPosition, inputVelocity, deltaT); 
    }
};

} // namespace subsystems
