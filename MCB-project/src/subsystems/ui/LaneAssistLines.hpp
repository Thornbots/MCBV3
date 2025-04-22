#pragma once

#include "subsystems/ui/UISubsystem.hpp"
#include "util/ui/GraphicsContainer.hpp"
#include "util/ui/SimpleGraphicsObjects.hpp"

using namespace tap::communication::serial;
using namespace subsystems;

// looks like / \ at the bottom of the screen
class LaneAssistLines : public GraphicsContainer {
public:
    LaneAssistLines() {
        addGraphicsObject(&left);
        addGraphicsObject(&right);
    }

    void update() {
        float pitch = gimbal ? gimbal->getPitchEncoderValue() : 0;
        float bottomOffset = A_BOTTOM * std::sin(B_BOTTOM * pitch + C_BOTTOM) + D_BOTTOM;
        float topOffset = A_TOP * std::sin(B_TOP * pitch + C_TOP) + D_TOP;

        left.x1 = static_cast<uint16_t>(UISubsystem::HALF_SCREEN_WIDTH - bottomOffset);
        left.x2 = static_cast<uint16_t>(UISubsystem::HALF_SCREEN_WIDTH - topOffset);
        right.x1 = static_cast<uint16_t>(UISubsystem::HALF_SCREEN_WIDTH + bottomOffset);
        right.x2 = static_cast<uint16_t>(UISubsystem::HALF_SCREEN_WIDTH + topOffset);
    }

    void setGimbalSubsystem(GimbalSubsystem* g) { gimbal = g; }

private:
#if defined(HERO)        // todo
    static constexpr uint16_t HEIGHT = 360;  // distance from bottom of the screen to the top of each line, pixels
    static constexpr float A_BOTTOM = 811.57925;
    static constexpr float B_BOTTOM = 1;
    static constexpr float C_BOTTOM = -2.2421;
    static constexpr float D_BOTTOM = 1019.47769;
    static constexpr float A_TOP = 985.39951;
    static constexpr float B_TOP = 1;
    static constexpr float C_TOP = -2.13925;
    static constexpr float D_TOP = 966.4288;
#elif defined(SENTRY)    // todo
    static constexpr uint16_t HEIGHT = 360;  // distance from bottom of the screen to the top of each line, pixels
    static constexpr float A_BOTTOM = 811.57925;
    static constexpr float B_BOTTOM = 1;
    static constexpr float C_BOTTOM = -2.2421;
    static constexpr float D_BOTTOM = 1019.47769;
    static constexpr float A_TOP = 985.39951;
    static constexpr float B_TOP = 1;
    static constexpr float C_TOP = -2.13925;
    static constexpr float D_TOP = 966.4288;
#elif defined(INFANTRY)  // https://www.desmos.com/calculator/xgwximvvmn
    static constexpr uint16_t HEIGHT = 360;  // distance from bottom of the screen to the top of each line, pixels
    static constexpr float A_BOTTOM = 811.57925;
    static constexpr float B_BOTTOM = 1;
    static constexpr float C_BOTTOM = -2.2421;
    static constexpr float D_BOTTOM = 1019.47769;
    static constexpr float A_TOP = 985.39951;
    static constexpr float B_TOP = 1;
    static constexpr float C_TOP = -2.13925;
    static constexpr float D_TOP = 966.4288;
#else                    // old infantry, todo
    static constexpr uint16_t HEIGHT = 360;  // distance from bottom of the screen to the top of each line, pixels
    static constexpr float A_BOTTOM = 811.57925;
    static constexpr float B_BOTTOM = 1;
    static constexpr float C_BOTTOM = -2.2421;
    static constexpr float D_BOTTOM = 1019.47769;
    static constexpr float A_TOP = 985.39951;
    static constexpr float B_TOP = 1;
    static constexpr float C_TOP = -2.13925;
    static constexpr float D_TOP = 966.4288;
#endif

    static constexpr uint16_t THICKNESS = 2;  // pixels

    GimbalSubsystem* gimbal = nullptr;

    Line left{RefSerialData::Tx::GraphicColor::CYAN, 0, 0, 0, HEIGHT, THICKNESS};
    Line right{RefSerialData::Tx::GraphicColor::CYAN, 0, 0, 0, HEIGHT, THICKNESS};
};