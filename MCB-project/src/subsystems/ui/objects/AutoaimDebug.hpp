#pragma once

#include <algorithm>
#include <cmath>

#include "subsystems/jetson/AutoAimAndFireCommand.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/AtomicGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class AutoaimDebug : public GraphicsContainer {
public:
    explicit AutoaimDebug(GimbalSubsystem* gimbal) : gimbal(gimbal) {
        addGraphicsObject(&yawdiff);
        addGraphicsObject(&pitchdiff);
        addGraphicsObject(&point);
    }

    void update(commands::AutoAimAndFireCommand* aaafc) {
        // Errors are measured minus requested, in radians; yaw uses the IMU boot frame.
        yawdiff._float = std::remainder(gimbal->getYawAngleRelativeWorld() - aaafc->targetYaw, 2 * PI);
        pitchdiff._float = gimbal->getPitchEncoderValue() - aaafc->targetPitch;
        const bool haveAim = aaafc->getIsScheduled() && aaafc->targeting;
        pitchdiff.setHidden(!haveAim);
        yawdiff.setHidden(!haveAim);

        // Height indicator at 100 pixels/metre, not a camera projection.
        point.cx = UISubsystem::HALF_SCREEN_WIDTH;
        const float y = UISubsystem::HALF_SCREEN_HEIGHT + aaafc->deltaZ * 100.0f;
        point.cy = std::isfinite(y)
            ? static_cast<uint16_t>(std::clamp(y, static_cast<float>(point.r), static_cast<float>(UISubsystem::SCREEN_HEIGHT - point.r)))
            : UISubsystem::HALF_SCREEN_HEIGHT;
        // Allow several UI batches; this still does not acknowledge client receipt.
        point.setHidden(!aaafc->getIsScheduled() || !aaafc->everRequestedFire || !std::isfinite(y) ||
                        tap::arch::clock::getTimeMilliseconds() - aaafc->lastFireRequestMs > 500);
    }

private:
    GimbalSubsystem* gimbal;

    FloatGraphic yawdiff{UISubsystem::Color::GREEN, 0.0f, 870, 360, 80, 4};
    FloatGraphic pitchdiff{UISubsystem::Color::GREEN, 0.0f, 1070, 620, 80, 4};
    UnfilledCircle point{UISubsystem::Color::ORANGE, 0, 0, 10, 1};
};