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
        addGraphicsObject(&point);
    }

    void update() {
        yawdiff._float = gimbal->getYawAngleRelativeWorld()-aaafc->targetYaw;
        pitchdiff._float = gimbal->getPitchEncoderValue()-aaafc->targetPitch;
        
        Vector2d xy = {aaafc->deltaX, aaafc->deltaY};
        xy.rotate(-gimbal->getYawAngleRelativeWorld());
        Vector3d position = {xy.getX(), xy.getY(), aaafc->deltaZ};
        Vector3d position2 = Projections::robotSpaceToPivotSpace(position);
        position = Projections::pivotSpaceToVtmSpace(position2);
        Vector2d screenPosition = Projections::vtmSpaceToScreenSpace(position);
        point.cx = screenPosition.getX();
        point.cy = screenPosition.getY();
        
        point.setHidden((aaafc->cvTarget.flags & CV_TARGET_FLAG_FIRE)==0 || !aaafc->receivedCvTargetEver);
        pitchdiff.setHidden(!aaafc->receivedCvTargetEver);
        yawdiff.setHidden(!aaafc->receivedCvTargetEver);
    }

private:
    GimbalSubsystem* gimbal;
    commands::AutoAimAndFireCommand* aaafc;

    FloatGraphic yawdiff{UISubsystem::Color::GREEN, 0.0f, 870, 360, 80, 4};
    FloatGraphic pitchdiff{UISubsystem::Color::GREEN, 0.0f, 1070, 620, 80, 4};
    UnfilledCircle point{UISubsystem::Color::GREEN, 0, 0, 10, 1};
};