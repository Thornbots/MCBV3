#include "ChassisController.hpp"
#include "ChassisControllerConstants.hpp"

#include <cmath>
#include <iostream>
#include <string>
// TODO: Calculate function that takes in target x, y velocities as well as a target angle as inputs, outputs 4 motor current summations

namespace subsystems {
ChassisController::ChassisController() {
    targetVelocityHistory = new float[Q_SIZE];
    forceHistory = new Pose2d[Q_SIZE];
    targetVelocityMagnitudeHistory = new float[BBQ_SIZE];
}

ChassisController::~ChassisController() {
    delete[] targetVelocityHistory;
    delete[] forceHistory;
    delete[] targetVelocityMagnitudeHistory;
}


// Function to calculate chassis state using historical force/torque data and latency THIS HAS PASSED TESTS
void ChassisController::estimateState(Pose2d inputForces, Pose2d* estVelLocal, Pose2d* estVelWorld, Pose2d* estPosWorld) {
    // For each historical value of Fx, Fy, Tz, calculate the estimates
    for (int i = Q_SIZE - 1; i >= 0; i--) {
        // // Store the current values in the history
        // load in new value if i is not > 0
        // forcehistory[Q_SIZE-1] gets kicked out every method call
        forceHistory[i] = (i > 0) ? forceHistory[i - 1] : inputForces;

        // Estimated angular velocity (theta_dot)
        estVelWorld->orientation() += forceHistory[i].getRotation() * (DT / J_EFFECTIVE);

        // Update theta estimate (added DT because i think alex Y did it wrong)
        estPosWorld->orientation() += estVelWorld->getRotation() * DT;

        // Update velocity
        estVelWorld->vec() += (forceHistory[i].rotate(estPosWorld->getRotation()) * (DT / M_EFFECTIVE));

        // update positions
        *estPosWorld += *estVelWorld * DT;
    }

    *estVelLocal = estVelWorld->rotate(-estPosWorld->getRotation());
}

float ChassisController::calculateBeybladeVelocity(float bb_freq, float bb_amp, Pose2d TargetVelocity) {
    // Get the target velocity commands (or position commands) from the controller
    // This part would integrate the user input or other controller logic to set target velocity
        float velmagMax = 0;
    for (int i = 0; i < BBQ_SIZE-1; i++) {
        targetVelocityMagnitudeHistory[i] = (i < (BBQ_SIZE - 2)) ? targetVelocityMagnitudeHistory[i + 1] : TargetVelocity.magnitude();
        velmagMax = std::max(velmagMax, targetVelocityMagnitudeHistory[i]);  // Find max velocity magnitude in the last bb_delay seconds
    }
    if (TargetVelocity.getRotation() < BBterm1 - 2) return TargetVelocity.getRotation(); //prevent any actual changes if the thing is slow (aka a PID controller or something is controlling positoin)
    // If fixed-speed beyblade or variable-speed beyblade with velocity != 0
    // Variable-speed beyblade behavior when velmagMax == 0
    // if (BEYBLADE_FIXED_SPEED || velmagMax != 0)
    //     dotThetaBeyblade -= velmagMax * dotThetaGain;
    // else if (!BEYBLADE_FIXED_SPEED && velmagMax == 0)
    //     dotThetaBeyblade -= (bb_amp / 2) + sawtooth(bb_freq, bb_amp);
    // else
    //     dotThetaBeyblade = 0;

    // Update target velocity queue
    // targetVelocityQueue.emplace_back(VEL_TARGET);
    // if (targetVelocityQueue.size() > BEYBLADE_DELAY / DT) {
    //     targetVelocityQueue.pop_front();
    // }
    return std::min(std::clamp(BBterm1 + BBterm2 * velmagMax + BBterm3 * velmagMax * velmagMax, 0.0f, BBmax), TargetVelocity.getRotation());  // Return the required angular velocity (dot_theta_req)
}

// Function to estimate input errors in inertial frame
void ChassisController::velocityControl(Pose2d inputVelLocal, Vector2d estVelWorld, Pose2d estVelLocal, Vector2d lastForceWorld, Pose2d* reqForceLocal) {
    Vector2d estForceInertial = (estVelWorld - lastVelWorld) * (M_EFFECTIVE / DT);

    // Compute the error between the last and the estimated forces in the inertial frame
    Vector2d errForceInertial = (lastForceWorld - estForceInertial);

    // Update integral error terms
    accumForceLocal += (errForceInertial - accumForceLocal) * KI_V * DT;

    //clamp accumulation
    accumForceLocal = accumForceLocal.clamp(MIN_FORCE, MAX_FORCE);

    // update the required local force (for all 3 elements)
    *reqForceLocal = Pose2d((inputVelLocal .getX() - estVelLocal.getX()) * KP_V_XY, (inputVelLocal .getY() - estVelLocal.getY()) * KP_V_XY,  std::clamp((inputVelLocal .getRotation() - estVelLocal.getRotation()) * KP_V_ROT, -maxTorqueZ, maxTorqueZ));//(inputVelLocal - estVelLocal) * KP_V;// + accumForceLocal;

    // Store current forces for the next iteration
    lastVelWorld = estVelWorld;
}

// Get the feed forward variables for each motor based on the previously calculaed inverse kinematic estimated theta (see feedforwards)
void ChassisController::calculateFeedForward(float estimatedMotorVelocity[4], float V_m_FF[4], float I_m_FF[4]) {
    float vel;
    for (int i = 0; i < 4; ++i) {
        vel = estimatedMotorVelocity[i];
        V_m_FF[i] = K_V * vel;
        I_m_FF[i] = K_VIS * vel + K_S * signum(vel);
    }
}
float motorTorque[4], forceArr[3], F_lat[4];
void ChassisController::calculateTractionLimiting(Pose2d localForce, Pose2d* limitedForce, float thetaDotDes) {

    multiplyMatrices(4, 3, forceInverseKinematics, localForce.toArray(forceArr), motorTorque);

    float largest = motorTorque[0];

    // Manipulate F_largest based on whether the beyblade torque command is positive or negative
    for (int i = 1; i < 4; i++) {  // if beybladeCommand > 0, find smallest, vice versa
        if (thetaDotDes > 0) {
            largest = std::max(largest, motorTorque[i]);
        } else {
            largest = std::min(largest, motorTorque[i]);
        }
    }

    largest = std::fabs(largest);

    float F_too_much = std::max(std::fabs(largest) - F_MAX / 4  , 0.0f);  // clamp between 0 and infinity

    float T_req = localForce.getRotation();

    float T_req_throttled = signum(T_req) * std::min(std::fabs(T_req), 2 * TRACKWIDTH * std::max(std::abs(T_req) / (2 * TRACKWIDTH) - F_too_much, F_MIN_T));

    // 2 rows to not do torque
    forceArr[2] = 0;
    multiplyMatrices(4, 3, forceInverseKinematics, forceArr, F_lat);

    largest = std::fabs(F_lat[0]);

    // Manipulate F_largest based on whether the beyblade torque command is positive or negative
    for (int i = 1; i < 4; i++) {  // if beybladeCommand > 0, find smallest, vice versa
        if (std::fabs(F_lat[i]) > largest) {
            largest = std::fabs(F_lat[i]);
        }
    }
    //start by settig the limited force to the local force
    *limitedForce = localForce;

    // set the limited force to the force, then multiply just the vector by the scaling factor
    // since 1/0 is inf, 1/0.00000000000001 is inf. so onlt set if not zero. its also never negative
    //if (largest > 0) limitedForce->vec() *= clamp((F_MAX / 4 - T_req_throttled / (2 * TRACKWIDTH)) * 1 / largest, 0.0f, 1.0f);
    *limitedForce = Pose2d(limitedForce->getX(), limitedForce->getY(), T_req_throttled);
}

// Calculates scaling factor based on the equations in the notion
void ChassisController::calculatePowerLimiting(float powerLimit, float V_m_FF[4], float I_m_FF[4], float T_req_m[4], float T_req_m2[4], float thetaDotEst, float thetaDotDes) {
    // // Get all the summations out of the way first
    float aSum = 0, bSumFirst = 0, bSumSecond = 0, cSumFirst = 0, cSumSecond = 0;
    for (int i = 0; i < 4; i++) {
        aSum += T_req_m[i] * T_req_m[i];
        bSumFirst += I_m_FF[i] * T_req_m[i];
        bSumSecond += V_m_FF[i] * T_req_m[i];
        cSumFirst += I_m_FF[i] * I_m_FF[i];
        cSumSecond += I_m_FF[i] * V_m_FF[i];
    }

    // Then get a, b, c to calculate the scaling factor with
    float a = (RA / (KT * KT)) * aSum;                              // KT^-2 T^2
    float b = ((2 * RA) / KT) * bSumFirst + (1 / KT) * bSumSecond;  // KT^-1 T^1 IV^1
    float c = RA * cSumFirst + cSumSecond - powerLimit*(P_FOS + (1 - P_FOS)*std::clamp((1 - thetaDotEst / 8.5f), 0.0f, 1.0f)) + P_IDLE;                  // IV^2

    float s_scaling = 1.0f;
    if(a > 0 && b * b - 4 * a * c > 0) s_scaling =  clamp((-b + std::sqrt(b * b - 4 * a * c)) / (2 * a), 0.0f, 1.0f);

    // And finally get T_req_m_throttled based on the scaling factor
    for (int i = 0; i < 4; ++i) {
        T_req_m2[i] = s_scaling * T_req_m[i];
    }
}

void ChassisController::followPosition(Vector2d targetPosition, Pose2d currentPosition, Pose2d inputVelocity, float powerLimit, float angle, float motorVelocity[4], float motorCurrent[4]){
    Vector2d controlEffort = (targetPosition - currentPosition.vec()) * KP;
    


    float normMagnitude = std::max(controlEffort.magnitude()/MAX_POS_VEL, 1.0f);

    Pose2d targetVelocity = inputVelocity + controlEffort * (1/normMagnitude); // Normalize the control effort to get the target velocity
  
    // Calculate the target velocity in the local frame
    calculate(targetVelocity.rotate(currentPosition.getRotation()), powerLimit, angle, motorVelocity, motorCurrent);
}
float estVelArr[3], estMotorArr[4];
void ChassisController::calculate(Pose2d targetVelLocal, float powerLimit, float angle, float motorVelocity[4], float motorCurrent[4]) {
    multiplyMatrices(3, 4, forwardKinematics, motorVelocity, estVelArr);
    // return;
    Pose2d estVelLocal(estVelArr);

    //also feed in odometry but this will kind of estimate 
    estPosWorld.orientation() = angle;

    estVelWorld = estVelLocal.rotate(angle);

    //TESTED
    estimateState(lastForceLocal, &estVelLocal, &estVelWorld, &estPosWorld);

    estVelLocal.toArray(estVelArr);

    // inverse kinematics
    multiplyMatrices(4, 3, inverseKinematics, estVelArr, estMotorArr);

    float V_m_FF[4], I_m_FF[4];  // motor feedforwards

    calculateFeedForward(estMotorArr, V_m_FF, I_m_FF); //technically inverse dynamics

    // get last inertial force too
    Pose2d lastForceWorld = lastForceLocal.rotate(estPosWorld.getRotation());

    //new force local
    Pose2d forceLocal;

    targetVelLocal = Pose2d(targetVelLocal.getX(), targetVelLocal.getY(), calculateBeybladeVelocity(0, 0, targetVelLocal));
    // // First, estimate the input errors then do velocity PI control
    velocityControl(targetVelLocal, estVelWorld, estVelLocal, lastForceWorld, &forceLocal);

    //calculateTractionLimiting(forceLocal, &forceLocal, targetVelLocal.getRotation());

    Pose2d forceTimes = forceLocal * (R_WHEEL / GEAR_RATIO);
    
    forceTimes.toArray(forceArr);

    multiplyMatrices(4, 3, forceInverseKinematics, forceArr, estMotorArr);

    calculatePowerLimiting(powerLimit, V_m_FF, I_m_FF, estMotorArr, estMotorArr, estVelLocal.getRotation(), targetVelLocal.getRotation());

    //have to reassign bc of how this works. Hopefully this gets garbage collected correctly
    lastForceLocal = Pose2d(multiplyMatrices(3, 4, forceKinematics, estMotorArr, forceArr));

    // set motor currents
    for (int i = 0; i < 4; i++) {
        
        motorCurrent[i] = estMotorArr[i] / KT + I_m_FF[i];
    }
}

}  // namespace subsystems
