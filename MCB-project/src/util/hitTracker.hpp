#pragma once

#include <functional>

#include "tap/communication/serial/ref_serial_data.hpp"
#include "util/HitOrientation.hpp"

#define RefSerialData tap::communication::serial::RefSerialData

class HitTracker
{
public:
    float hitOrientation{0.0f};
    bool isHit{false};
    uint16_t previousHp{0};

    static constexpr float PLACEHOLDER_ANGLE = 123.0f;

    explicit HitTracker(tap::Drivers* drivers) : drivers(drivers) {}

    void setGetYawEncoderFunction(std::function<float()> func)
    {
        getYawEncoderValue_func = func;
    }

    void addHit()
    {
        const float yawEncoder = getYawEncoderValue_func();
        const float headWorldYaw = drivers->bmi088.getYaw();
        const uint8_t damagedArmorId = static_cast<uint8_t>(drivers->refSerial.getRobotData().damagedArmorId);
        hitOrientation = hit_orientation::worldYaw(headWorldYaw, yawEncoder, damagedArmorId);
        isHit = true;
    }

    void update()
    {
        isHit = false;

        if (drivers->refSerial.getRefSerialReceivingData() && getYawEncoderValue_func) {
            const RefSerialData::Rx::RobotData& robotData = drivers->refSerial.getRobotData();
            if (previousHp > robotData.currentHp &&
                (robotData.damageType == RefSerialData::Rx::DamageType::ARMOR_DAMAGE ||
                 robotData.damageType == RefSerialData::Rx::DamageType::COLLISION)) {
                addHit();
            }

            previousHp = robotData.currentHp;
        }
    }

    float getAngleToTurnForSentry() const
    {
        return isHit ? hitOrientation : PLACEHOLDER_ANGLE;
    }

    float getAngleForRefSys(float currentChassisWorldYaw) const
    {
        return isHit ? hit_orientation::relativeYaw(hitOrientation, currentChassisWorldYaw) : PLACEHOLDER_ANGLE;
    }

private:
    tap::Drivers* drivers;
    std::function<float()> getYawEncoderValue_func;
};
