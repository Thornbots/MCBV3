#pragma once
#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/servo.hpp"

#include "communication/JetsonCommunication.hpp"

#include "drivers.hpp"

using namespace communication;
using namespace tap::algorithms::ballistics;

namespace subsystems {
struct PanelData {
    double r;
    double theta;
};

class ComputerVisionSubsystem : public tap::control::Subsystem {
private:  // Private Variables
    JetsonCommunication& comm;
    src::Drivers* drivers;

public:  // Public Methods
    ComputerVisionSubsystem(src::Drivers* drivers, JetsonCommunication& comm);

    ~ComputerVisionSubsystem() {}

    void initialize();

    void refresh() override;

    void update(float current_pitch, float current_yaw, float* yawOut, float* pitchOut, int* action);


private:  // Private Methods

    // Constants
    const float g = 9.81;           // gravitational acceleration
    const float J = 25.0;           // Shot velocity
    const float l = 0.05;           // Combined camera + Jetson latency
    const float deltaTime = 0.033;  // Frame time
    const float H = 9.95;           // Height rejection offset
    std::vector<PanelData> panelData;
};
}  // namespace subsystems