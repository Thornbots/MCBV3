#include "MouseMoveCommand.hpp"
#include "GimbalSubsystemConstants.hpp"

namespace commands {

void MouseMoveCommand::initialize() {
}
void MouseMoveCommand::execute() {
    yaw = MOUSE_YAW_PROPORTIONAL * (drivers->remote.getMouseX());
    pitch += MOUSE_PITCH_PROPORTIONAL * (drivers->remote.getMouseY());

#if defined(INFANTRY)
    pitch -= drivers->bmi088.getRoll() * PI / 180;
#elif defined(HERO)
    // todo for hero lmao
#endif

    gimbal->updateMotors(yaw, &pitch);
}

void MouseMoveCommand::end(bool) { pitch = 0; }

bool MouseMoveCommand::isFinished() const { return !drivers->remote.isConnected(); }
}  // namespace commands