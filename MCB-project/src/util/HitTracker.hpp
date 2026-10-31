#pragma once

#include <functional>

#include "tap/communication/serial/ref_serial_data.hpp"
#include "util/SentryBuzzer.hpp"
#include "util/HitOrientation.hpp"
#include "util/HitEventState.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"

// true makes the robot beep when it gets hit, false means it silently endures it
#define FEELS_PAIN false

using RefSerialData = tap::communication::serial::RefSerialData;

class HitTracker
{
public:
    uint16_t previousHp{0};

    static constexpr float PLACEHOLDER_ANGLE = hit_orientation::NO_HIT_ANGLE;

    explicit HitTracker(tap::Drivers* drivers, src::SentryBuzzer* buzzer = nullptr)
        : drivers(drivers), buzzer(buzzer) {}

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
            if (hasPreviousHp && previousHp > robotData.currentHp) {
                addHit();
#if FEELS_PAIN
                if (buzzer) buzzer->setHit(true);
                else tap::buzzer::playNote(&(drivers->pwm), 4000);
                painTimeout.restart(250);
#endif
            }

#if FEELS_PAIN
            if (painTimeout.execute()) {
                if (buzzer) buzzer->setHit(false);
                else tap::buzzer::silenceBuzzer(&(drivers->pwm));
            }
#endif

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
    src::SentryBuzzer* buzzer;
    std::function<float()> getYawEncoderValue_func;
    bool hasPreviousHp{false};
    HitEventState hitState;
    
#if FEELS_PAIN
    tap::arch::MilliTimeout painTimeout;
#endif
};
