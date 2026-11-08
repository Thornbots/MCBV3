// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "util/HitOrientation.hpp"

class HitEventState
{
public:
    // The control event expires each cycle; the latest bearing survives until reported.
    void record(float hitWorldYaw)
    {
        hitWorldYaw_ = hitWorldYaw;
        hitThisControlCycle_ = true;
        reportPending_ = true;
    }

    void beginControlCycle()
    {
        hitThisControlCycle_ = false;
    }

    bool hitThisControlCycle() const
    {
        return hitThisControlCycle_;
    }

    float hitWorldYawThisControlCycle() const
    {
        return hitThisControlCycle_ ? hitWorldYaw_ : hit_orientation::NO_HIT_ANGLE;
    }

    float angleForRefSys(float currentChassisWorldYaw) const
    {
        return reportPending_ ? hit_orientation::relativeYaw(hitWorldYaw_, currentChassisWorldYaw)
                              : hit_orientation::NO_HIT_ANGLE;
    }

    void consumeReport()
    {
        reportPending_ = false;
    }

private:
    float hitWorldYaw_{0.0f};
    bool hitThisControlCycle_{false};
    bool reportPending_{false};
};
