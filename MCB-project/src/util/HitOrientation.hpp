#pragma once

#include <cmath>
#include <cstdint>

namespace hit_orientation {
constexpr float TWO_PI = 6.2831853071795864769f;

constexpr float armorOffset(uint8_t damagedArmorId)
{
    return damagedArmorId < 4 ? static_cast<float>(damagedArmorId) * (TWO_PI / 4.0f) : 0.0f;
}

constexpr float worldYaw(float headWorldYaw, float yawEncoder, uint8_t damagedArmorId)
{
    return headWorldYaw - yawEncoder + armorOffset(damagedArmorId);
}

inline float relativeYaw(float hitWorldYaw, float currentChassisWorldYaw)
{
    return std::remainder(hitWorldYaw - currentChassisWorldYaw, TWO_PI);
}
}
