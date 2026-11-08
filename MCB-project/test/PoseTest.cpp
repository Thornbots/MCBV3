#include <gtest/gtest.h>
#include <cmath>
#include <util/Pose2d.hpp>


// Test Fixture
class Pose2dTest : public ::testing::Test {
protected:
    Pose2d p1{3.0f, 4.0f, 1.57f}; // x=3, y=4, rotation=~90 degrees
    Pose2d p2{1.0f, 2.0f, 0.78f}; // x=1, y=2, rotation=~45 degrees
};

// Test default constructor
TEST_F(Pose2dTest, DefaultConstructor) {
    Pose2d p;
    EXPECT_EQ(p, Pose2d(0.0f, 0.0f, 0.0f));
}

// Test parameterized constructor
TEST_F(Pose2dTest, ParameterizedConstructor) {
    EXPECT_EQ(p1, Pose2d(3.0f, 4.0f, 1.57f));
}

// Test vector constructor
TEST_F(Pose2dTest, VectorConstructor) {
    Pose2d p(p1.vec());
    EXPECT_EQ(p, Pose2d(3.0f, 4.0f, 0.0f));
}

// Test pose constructor
TEST_F(Pose2dTest, PoseConstructor) {
    Pose2d p(p1);
    EXPECT_EQ(p, p1);
}

// Test array-based constructor
TEST_F(Pose2dTest, ArrayConstructor) {
    float arr[3] = {2.0f, 5.0f, 1.0f};
    Pose2d p(arr);
    EXPECT_EQ(p, Pose2d(2.0f, 5.0f, 1.0f));
}

// Test equality operator
TEST_F(Pose2dTest, EqualityOperator) {
    Pose2d p3(3.0f, 4.0f, 1.57f);
    EXPECT_EQ(p1, p3);
    EXPECT_FALSE(p1 == p2);
}

// Test addition operator
TEST_F(Pose2dTest, AdditionOperator) {
    Pose2d result = p1 + p2;
    EXPECT_EQ(result, Pose2d(4.0f, 6.0f, 1.57f));
}

// Test subtraction operator
TEST_F(Pose2dTest, SubtractionOperator) {
    Pose2d result = p1 - p2;
    EXPECT_EQ(result, Pose2d(2.0f, 2.0f, 1.57f));
}

// Test scalar multiplication operator
TEST_F(Pose2dTest, ScalarMultiplicationOperator) {
    Pose2d result = p1 * 2.0f;
    EXPECT_EQ(result, Pose2d(6.0f, 8.0f, 3.14f));
}

// Test addition-assignment operator
TEST_F(Pose2dTest, AdditionAssignmentOperator) {
    Pose2d temp = p1;
    temp += p2;
    EXPECT_EQ(temp, Pose2d(4.0f, 6.0f, 1.57f));
}

// Test subtraction-assignment operator
TEST_F(Pose2dTest, SubtractionAssignmentOperator) {
    Pose2d temp = p1;
    temp -= p2;
    EXPECT_EQ(temp, Pose2d(2.0f, 2.0f, 1.57f));
}

// Test scalar multiplication-assignment operator
TEST_F(Pose2dTest, ScalarMultiplicationAssignmentOperator) {
    Pose2d temp = p1;
    temp *= 2.0f;
    EXPECT_EQ(temp, Pose2d(6.0f, 8.0f, 3.14f));
}

// Test assignment operator
TEST_F(Pose2dTest, AssignmentOperator) {
    Pose2d p3;
    p3 = p1;
    EXPECT_EQ(p3, p1);
}
// Test rotation function
TEST_F(Pose2dTest, RotateFunction) {
    Pose2d rotated = p1.rotate(3.14159265f/2.0f); // Rotate by 90 degrees
    EXPECT_EQ(rotated, Pose2d(-4.0f, 3.0f, 1.57f));
}

// Test adding rotation function
TEST_F(Pose2dTest, AddJustRotationFunction) {
    Pose2d result = p1;
    result.orientation() += 0.5f;
    EXPECT_EQ(result, Pose2d(3.0f, 4.0f, 2.07f));
}

// // Test array
// TEST_F(Pose2dTest, ArrayFunction) {
//     float* result = p1;
//     EXPECT_EQ(result[0], p1.getX());
//     EXPECT_EQ(result[1], p1.getY());
//     EXPECT_EQ(result[2], p1.getRotation());
// }

// // Test array but on a vector
// TEST_F(Pose2dTest, ArrayVectorFunction) {
//     float* result = p1.vec();
//     EXPECT_EQ(result[0], p1.getX());
//     EXPECT_EQ(result[1], p1.getY());
// }


//tests for making sure vectors dont screw things up

TEST_F(Pose2dTest, vectest1) {
    Pose2d result = p1.vec();
    EXPECT_EQ(result, Pose2d(3.0f, 4.0f, 0.0f));
}

TEST_F(Pose2dTest, vectest2) {
    Pose2d result;
    result.vec() = p1;

    EXPECT_EQ(result, Pose2d(3.0f, 4.0f, 0.0f));
}
TEST_F(Pose2dTest, otest1) {
    Pose2d result = p1;
    result.orientation() = 0.0f;
    
    EXPECT_EQ(result, Pose2d(3.0f, 4.0f, 0.0f));
}

TEST_F(Pose2dTest, otest2) {
    Pose2d result = p1;
    Pose2d* ptr = &result;

    ptr->orientation() = 0.0f;
    
    EXPECT_EQ(result, Pose2d(3.0f, 4.0f, 0.0f));
}

TEST_F(Pose2dTest, passingtest) {
    Pose2d result = p1;
    Vector2d* ptr = &result;

    *ptr *= 0.0f; //zero out vector not the pose. this operator on the pose will zero it out
    
    EXPECT_EQ(result, Pose2d(0.0f, 0.0f, 1.57));
}

TEST_F(Pose2dTest, clamptest) {
    Pose2d minp{-1, -1, -1};
    Pose2d maxp{1, 1, 1};

    EXPECT_EQ(p1.clamp(minp, maxp), Pose2d(1,1,1));

}
