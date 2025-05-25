#pragma once
#include <cmath>
#include <iostream>
#include <string>

#include "util/Pose2d.hpp"

namespace subsystems {
// REMEMBER THAT YOU USE NEGATIVE FOR INERTIAL TO LOCAL AND POSITIVE FOR LOCAL TO INERTIAL
class ChassisController {
private:

    // Beyblade settings
    float dotThetaBeyblade = 0;  // Target beyblade velocity when chassis is not translating
    float dotThetaGain = 0;      // Amount to subtract from dot_theta_beyblade per meter/s of chassis speed

    float signum(float num) { return (num > 0) ? 1 : ((num < 0) ? -1 : 0); }

    float clamp(float num, float min, float max) { return std::min(std::max(num, min), max); }

    // Helper for velocity control (sawtooth function)
    float sawtooth(__attribute__((unused)) float freq, float amplitude) {
        // A simple sawtooth wave function for variable speed beyblade
        return amplitude;  // * (1 - fmod(freq * (std::fmod(std::clock(), 1.0f) * 2 * 3.14159265f), 1.0f));
    }

        // states to store, current position, curent velocity, and also the lastForce
    Pose2d estPosWorld{}, estVelWorld{}, lastForceLocal{};

    Pose2d lastVelWorld{}, accumForceLocal{};

public:
    float *targetVelocityHistory;  // For storing target velocity magnitudes
    Pose2d *forceHistory;          // history of past chassis forces
    float *targetVelocityMagnitudeHistory; //used for beyblade gain calculations
    ChassisController();

    ~ChassisController();
    //~YawController();
    void calculate(Pose2d targetVelLocal, float powerLimit, float angle, float motorVelocity[4], float motorCurrent[4]);

    void followPosition(Vector2d targetPosition, Pose2d currentPosition, Pose2d inputVelocity, float powerLimit, float angle, float motorVelocity[4], float motorCurrent[4]);
        
    float calculateBeybladeVelocity(float bb_freq, float bb_amp, Pose2d TargetVelocity);

    // intermediate functions
    void estimateState(Pose2d inputForces, Pose2d *eLocalVel, Pose2d *eInertialVel, Pose2d *eInertialPos);

    void calculateFeedForward(float estimatedMotorVelocity[4], float V_m_FF[4], float I_m_FF[4]);

    void velocityControl(Pose2d inputLocalVel, Vector2d estInertialVel, Pose2d estLocalVel, Vector2d lastInertialForce, Pose2d *reqLocalForce);

    void calculateTractionLimiting(Pose2d localForce, Pose2d *limitedForce, float thetaDotDes);

    void calculatePowerLimiting(float powerLimit, float V_m_FF[4], float I_m_FF[4], float T_req_m[4], float T_req_m2[4], float thetaDotEst, float thetaDotDes);

    float *multiplyMatrices(int rows1, int cols1, const float **mat1, float *mat2, float *result) {
        // Iterate over rows of mat1 and columns of mat2
        for (int i = 0; i < rows1; i++) {
            result[i] = 0.0f;  // Initialize to zero before accumulating
            for (int j = 0; j < cols1; j++) {
                result[i] += mat1[i][j] * mat2[j];
            }
        }
        return result;
    }


};

}  // namespace subsystems
