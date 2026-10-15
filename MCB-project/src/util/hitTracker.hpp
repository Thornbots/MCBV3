#pragma once

#include <functional>

#include "tap/communication/serial/ref_serial_data.hpp"
#include "util/HitOrientation.hpp"
#include "util/HitEventState.hpp"

#define RefSerialData tap::communication::serial::RefSerialData

class HitTracker
{
public:
    uint16_t previousHp{0};

    static constexpr float PLACEHOLDER_ANGLE = hit_orientation::NO_HIT_ANGLE;

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
        recordHit(headWorldYaw, yawEncoder, damagedArmorId);
    }

    void recordHit(float headWorldYaw, float yawEncoder, uint8_t damagedArmorId)
    {
        hitState.record(hit_orientation::worldYaw(headWorldYaw, yawEncoder, damagedArmorId));
    }

    void beginControlCycle()
    {
        hitState.beginControlCycle();
    }

    void update()
    {
        beginControlCycle();

        if (drivers->refSerial.getRefSerialReceivingData() && getYawEncoderValue_func) {
            const RefSerialData::Rx::RobotData& robotData = drivers->refSerial.getRobotData();
            if (hasPreviousHp && previousHp > robotData.currentHp &&
                (robotData.damageType == RefSerialData::Rx::DamageType::ARMOR_DAMAGE ||
                 robotData.damageType == RefSerialData::Rx::DamageType::COLLISION)) {
                addHit();
            }

            previousHp = robotData.currentHp;
            hasPreviousHp = true;
        }
    }

    float getAngleToTurnForSentry() const
    {
        return hitState.hitWorldYawThisControlCycle();
    }

    bool didHitThisControlCycle() const
    {
        return hitState.hitThisControlCycle();
    }

    float getHitOrientation() const
    {
        return hitState.hitWorldYawThisControlCycle();
    }

    float getAngleForRefSys(float currentChassisWorldYaw) const
    {
        return hitState.angleForRefSys(currentChassisWorldYaw);
    }

    void consumeReportedHit()
    {
        hitState.consumeReport();
    }

private:
    tap::Drivers* drivers;
    std::function<float()> getYawEncoderValue_func;
    bool hasPreviousHp{false};
    HitEventState hitState;
};
