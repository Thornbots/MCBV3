#pragma once

#include "tap/control/command.hpp"

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"

#include "LaneAssistLines.hpp"
#include "drivers.hpp"

namespace commands {
using subsystems::UISubsystem;

class UIDrawCommand : public tap::control::Command, GraphicsContainer {
public:
    UIDrawCommand(UISubsystem* subsystem) : subsystem(subsystem) { addSubsystemRequirement(subsystem); }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "ui draw command"; }

private:
    UISubsystem* subsystem;

    // add top level graphics objects here and in initialize()
    LaneAssistLines laneAssistLines{};
};
}  // namespace commands