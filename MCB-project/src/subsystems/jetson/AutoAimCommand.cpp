#include "AutoAimCommand.hpp"

namespace commands {
void AutoAimCommand::initialize() {
    shoot = -1;
    MouseMoveCommand::initialize();
}
void AutoAimCommand::execute() {
    float dyaw = 0;
    float currentYaw = gimbal->getYawAngleRelativeWorld();
    float currentPitch = gimbal->getPitchEncoderValue();
    cv->update(currentYaw, currentPitch, yawvel, pitchvel, &dyaw, &pitch, &yawvel, &pitchvel, &shoot);
    // moving gimbal
    if (shoot != -1) {
        // moving to panel

        dyaw = fmod(dyaw, 2*PI);
        //clamp between -Pi and PI to allow for dividing
        dyaw = dyaw > PI ? dyaw - 2*PI : dyaw < -PI ? dyaw + 2*PI : dyaw;
        float newPitch = currentPitch + (pitch - currentPitch) / 2.0f;
        if (abs(dyaw) > .05) {
            dyaw /= (4.0f*2.5f);}
        else dyaw /= (1.75f*2.5f);
        gimbal->updateMotorsAndVelocityWithLatencyCompensation(dyaw, newPitch, yawvel, pitchvel); //division is to prevent overshoot from latency
        //gimbal->updateMotors(dyaw/3.0f, pitch);
    } else {
        // sentry's equivalent of patrol, do original mouse moving
        MouseMoveCommand::execute();
    }
}

void AutoAimCommand::end(bool interrupted) { MouseMoveCommand::end(interrupted); }

bool AutoAimCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands