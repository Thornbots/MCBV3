//=============================================================================================
// MahonyAHRS.h
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date          Author          Notes
// 29/09/2011    SOH Madgwick    Initial release
// 02/10/2011    SOH Madgwick    Optimised for reduced CPU load
// 09/06/2020    Matthew Arnold  Update style, use safer casting
//
//=============================================================================================
#ifndef MAHONY_AHRS_H_
#define MAHONY_AHRS_H_

#include <cmath>

//--------------------------------------------------------------------------------------------
// Variable declaration

class Mahony
{
private:
    float twoKp;           // 2 * proportional gain (Kp)
    float twoKi;           // 2 * integral gain (Ki)
    float q0, q1, q2, q3;  // quaternion of sensor frame relative to auxiliary frame
    float integralFBx, integralFBy, integralFBz;  // integral error terms scaled by Ki
    float invSampleFreq;
    float roll, pitch, yaw;
    static float invSqrt(float x);
    void computeAngles();

    //-------------------------------------------------------------------------------------------
    // Function declarations

public:
    Mahony();
    void begin(float sampleFrequency, float kp, float ki)
    {
        invSampleFreq = 1.0f / sampleFrequency;
        twoKp = 2.0f * kp;
        twoKi = 2.0f * ki;
    }
    void reset()
    {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
        roll = 0.0f;
        pitch = 0.0f;
        yaw = 0.0f;
    }
    void update(
        float gx,
        float gy,
        float gz,
        float ax,
        float ay,
        float az,
        float mx,
        float my,
        float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    float getRoll() const { return roll * 57.29578f; }
    float getPitch() const { return pitch * 57.29578f; }
    float getYaw() const
    {
        float yawDegrees = yaw * 57.29578f;
        return fmod(yawDegrees + 360.0f, 360.0f);
    }
    float getRollRadians() const { return roll; }
    float getPitchRadians() const { return pitch; }
    float getYawRadians() const { return yaw; }
    float getq0() const {return q0;}
    float getq1() const {return q1;}
    float getq2() const {return q2;}
    float getq3() const {return q3;}

};

#endif  // MAHONY_AHRS_H_
