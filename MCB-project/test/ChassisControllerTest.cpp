#include <gtest/gtest.h>
#include "subsystems/drivetrain/ChassisController.hpp"

namespace subsystems {
extern const float *inverseKinematics[4];
}

using namespace subsystems;

class ChassisControllerTest : public ::testing::Test {
protected:
    ChassisController controller;
    Pose2d localVel, inertialVel, inertialPos;
    void runForwardStatePass() {
        // Common initial conditions
        localVel = Pose2d(1, 2, 0);
        inertialVel = Pose2d(-0.6316, 2.1450, 3);
        inertialPos = Pose2d(0, 0, 0.75);

        controller.estimateState(Pose2d(6,7,7), &localVel, &inertialVel, &inertialPos);

    }

    void runLoadStateHistory() {
        for (const auto& pose : {Pose2d(1,1,3), Pose2d(2,2,4), Pose2d(3,3,7), Pose2d(4,4,7)}) {
            controller.estimateState(pose, &localVel, &inertialVel, &inertialPos);
        }
    }
};
class ChassisControllerMatrixTest : public ::testing::Test {
protected:
    ChassisController controller;
    Pose2d ikInputs[4] = {
        Pose2d(-0.2297, -0.2587, 7.1358),
        Pose2d(-0.2376, -0.2600, 7.1192),
        Pose2d(-0.3028, -0.2644, 6.9863),
        Pose2d(0, 0, 0)
    };

};

TEST_F(ChassisControllerTest, EstimateStateHistory) {
    runLoadStateHistory();

    for(int i = 0; i < 4; i++){
        EXPECT_NEAR(4-i, controller.forceHistory[i].getX(), 0.001); //if x is right they all are probably are
    }
}
TEST_F(ChassisControllerTest, EstimateStateHistory2) {
    runLoadStateHistory();
    runForwardStatePass();

    Pose2d expected = inertialVel.rotate(-inertialPos.getRotation());
    EXPECT_NEAR(expected.getX(), localVel.getX(), 0.0001f);
    EXPECT_NEAR(expected.getY(), localVel.getY(), 0.0001f);
    EXPECT_NEAR(inertialVel.getRotation(), localVel.getRotation(), 0.0001f);
    // Golden values for the sentry calibration selected by tools/test_cpp.sh.
    EXPECT_NEAR(1.0486532f, localVel.getX(), 0.0001f);
    EXPECT_NEAR(1.9762056f, localVel.getY(), 0.0001f);
    EXPECT_NEAR(3.0287876f, localVel.getRotation(), 0.0001f);
}
TEST_F(ChassisControllerTest, EstimateStateHistory3) {

    runLoadStateHistory();
    runForwardStatePass();

    // Positive local force rotated by 0.75 rad points left and forward.
    EXPECT_LT(inertialVel.getX(), -0.6316f);
    EXPECT_GT(inertialVel.getY(), 2.1450f);
    EXPECT_GT(inertialVel.getRotation(), 3.0f);
    EXPECT_NEAR(-0.6317f, inertialVel.getX(), 0.0001f);
    EXPECT_NEAR(2.1461873f, inertialVel.getY(), 0.0001f);
    EXPECT_NEAR(3.0287876f, inertialVel.getRotation(), 0.0001f);
}
TEST_F(ChassisControllerTest, EstimateStateHistory4) {

    runLoadStateHistory();
    runForwardStatePass();


    EXPECT_NEAR(-0.0051f, inertialPos.getX(), 0.0001f);
    EXPECT_NEAR(0.0172f, inertialPos.getY(), 0.0001f);
    EXPECT_NEAR(0.7740f, inertialPos.getRotation(), 0.0001f);
}


TEST_F(ChassisControllerMatrixTest, inverseTest){
    for(int i = 0; i < 4; i++){
        float arr[3] = {ikInputs[i].getX(), ikInputs[i].getY(), ikInputs[i].getRotation()};
        float test[4];
        controller.multiplyMatrices(4, 3, inverseKinematics, arr, test);
        for(int j = 0; j < 4; j++){
            EXPECT_TRUE(std::isfinite(test[j]));
        }
        EXPECT_NEAR((test[0] + test[2]) / 2, inverseKinematics[0][2] * arr[2], 0.03f);
        EXPECT_NEAR((test[1] + test[3]) / 2, inverseKinematics[1][2] * arr[2], 0.03f);
    }
}



// TEST_F(ChassisControllerTest, CalculateBeybladeVelocity) {
//     float vel = controller.calculateBeybladeVelocity(1.0, 2.0);
//     EXPECT_NEAR(vel, 0.0, 10.0); // Arbitrary range check
// }

// TEST_F(ChassisControllerTest, VelocityControl) {
//     Pose2d velError[] = {
//         Pose2d(0.4893, -0.6258, 1.9289e-04),
//         Pose2d(0.4773, -0.6243, 1.8013e-04),
//         Pose2d(0.4653, -0.6227, 1.6822e-04),
//         Pose2d(0.4534, -0.6209, 1.5709e-04),
//         Pose2d(0.4416, -0.6188, 1.4670e-04),
//         Pose2d(0.4299, -0.6166, 1.3700e-04)
//     };
//     Vector2d inputError[] = {
//         Vector2d(1.5687, -2.8793),
//         Vector2d(1.5327, -2.9039),
//         Vector2d(1.4967, -2.9284),
//         Vector2d(1.4606, -2.9527),
//         Vector2d(1.4243, -2.9769),
//         Vector2d(1.3880, -3.0009)
//     };

//     Pose2d outputForces[] {
//         Pose2d(1469.9, -1880.0, 0.0077),
//         Pose2d(1433.7, -1875.7, 0.0072),
//         Pose2d(1397.7, -1870.8, 0.0067),
//         Pose2d(1362.0, -1865.3, 0.0063),
//         Pose2d(1326.6, -1859.2, 0.0059),
//         Pose2d(1291.4, -1852.6, 0.0055)
//     };

//     controller.velocityControl(inputLocalVel, estInertialVel, estLocalVel, lastInertialForce, &reqLocalForce);
//     EXPECT_NEAR(reqLocalForce.getX(), 0.2, 0.1);
//     EXPECT_NEAR(reqLocalForce.getY(), 0.2, 0.1);
// }

TEST_F(ChassisControllerTest, CalculateFeedForward) {
    float estimatedMotorVelocity[4] = {1.0, 2.0, 3.0, 4.0};
    float V_m_FF[4], I_m_FF[4];
    controller.calculateFeedForward(estimatedMotorVelocity, V_m_FF, I_m_FF);
    for (int i = 0; i < 4; i++) {
        EXPECT_GT(V_m_FF[i], 0);
        EXPECT_GT(I_m_FF[i], 0);
    }
}

TEST_F(ChassisControllerTest, CalculateTractionLimiting) {
    Pose2d localForce(1.0, 1.0, 0.1);
    Pose2d limitedForce;
    controller.calculateTractionLimiting(localForce, &limitedForce, 0.0f);
    EXPECT_LE(limitedForce.getX(), localForce.getX());
    EXPECT_LE(limitedForce.getY(), localForce.getY());
}

TEST_F(ChassisControllerTest, CalculatePowerLimiting) {
    float V_m_FF[4] = {1.0, 1.2, 1.1, 1.3};
    float I_m_FF[4] = {0.5, 0.6, 0.55, 0.65};
    float T_req_m[4] = {2.0, 2.5, 2.2, 2.8};
    float T_req_m2[4];
    controller.calculatePowerLimiting(60, V_m_FF, I_m_FF, T_req_m, T_req_m2, 0.0f, 0.0f);
    for (int i = 0; i < 4; i++) {
        EXPECT_LE(T_req_m2[i], T_req_m[i]);
    }
}
