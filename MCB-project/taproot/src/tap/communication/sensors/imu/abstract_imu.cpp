/*****************************************************************************/
/********** !!! WARNING: CODE GENERATED BY TAPROOT. DO NOT EDIT !!! **********/
/*****************************************************************************/

/*
 * Copyright (c) 2020-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "abstract_imu.hpp"

namespace tap::communication::sensors::imu
{
void AbstractIMU::initialize(float sampleFrequency, float mahonyKp, float mahonyKi)
{
    mahonyAlgorithm.begin(sampleFrequency, mahonyKp, mahonyKi);
    imuState = ImuState::IMU_NOT_CALIBRATED;
    readTimeout.restart(1'000'000 / sampleFrequency);
}

void AbstractIMU::requestCalibration()
{
    if (imuState == ImuState::IMU_NOT_CALIBRATED || imuState == ImuState::IMU_CALIBRATED)
    {
        resetOffsets();
        calibrationSample = 0;
        imuState = ImuState::IMU_CALIBRATING;
    }
}

void AbstractIMU::setMountingTransform(const Transform& transform)
{
    mountingTransform = transform;
}

void AbstractIMU::periodicIMUUpdate()
{
    if (imuState == ImuState::IMU_CALIBRATING)
    {
        computeOffsets();
    }
    else
    {
        gyroDegPerSecTransformed = mountingTransform.getRotation().matrix() * imuData.gyroDegPerSec.coordinates();
        accGTransformed = mountingTransform.getRotation().matrix() * imuData.accG.coordinates();
        mahonyAlgorithm.updateIMU(
            gyroDegPerSecTransformed.data[0],
            gyroDegPerSecTransformed.data[1],
            gyroDegPerSecTransformed.data[2],
            accGTransformed.data[0],
            accGTransformed.data[1],
            accGTransformed.data[2]);
    }
}

void AbstractIMU::resetOffsets()
{
    imuData.accOffsetRaw = {0, 0, 0};
    imuData.gyroOffsetRaw = {0, 0, 0};
}

void AbstractIMU::setAccelOffset(float x, float y, float z)
{
    imuData.accOffsetRaw = tap::algorithms::transforms::Vector(x, y, z);
}

void AbstractIMU::setGyroOffset(float x, float y, float z)
{
    imuData.gyroOffsetRaw = tap::algorithms::transforms::Vector(x, y, z);
}

void AbstractIMU::computeOffsets()
{
    calibrationSample++;

    imuData.gyroOffsetRaw = imuData.gyroOffsetRaw + imuData.gyroRaw;
    imuData.accOffsetRaw = imuData.accOffsetRaw + imuData.accRaw -
                           tap::algorithms::transforms::Vector(0, 0, getAccelerationSensitivity());

    if (calibrationSample >= offsetSampleCount)
    {
        calibrationSample = 0;
        imuData.gyroOffsetRaw = imuData.gyroOffsetRaw / offsetSampleCount;
        imuData.accOffsetRaw = imuData.accOffsetRaw / offsetSampleCount;
        imuState = ImuState::IMU_CALIBRATED;
        mahonyAlgorithm.reset();
    }
}

}  // namespace tap::communication::sensors::imu
