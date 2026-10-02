#pragma once

#include "subsystems/jetson/AutoAimAndFireCommand.hpp"
#include "subsystems/gimbal/GimbalSubsystem.hpp"
#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/AtomicGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

class AutoaimDebug : public GraphicsContainer {
public:
    AutoaimDebug(GimbalSubsystem* gimbal, commands::AutoAimAndFireCommand* aaafc) : gimbal(gimbal), aaafc(aaafc) {
        addGraphicsObject(&yawdiff);
        addGraphicsObject(&pitchdiff);
    }

    void update() {
        yawdiff._float = gimbal->getYawAngleRelativeWorld()-aaafc->targetYaw;
        pitchdiff._float = gimbal->getPitchEncoderValue()-aaafc->targetPitch;
    }

private:
    GimbalSubsystem* gimbal;
    commands::AutoAimAndFireCommand* aaafc;

    FloatGraphic yawdiff{UISubsystem::Color::GREEN, 0.0f, 870, 360, 80, 4};
    FloatGraphic pitchdiff{UISubsystem::Color::GREEN, 0.0f, 1070, 520, 80, 4};
};