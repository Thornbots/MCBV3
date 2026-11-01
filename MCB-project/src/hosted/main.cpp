// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
// Hardware fixture for the real sentry firmware. stdin/stdout are lockstep IPC;
// UART1 is a raw PTY. All protocol parsing and control run in the MCB sources.
#include <array>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <vector>
#include <algorithm>
#include <chrono>
#include <sys/socket.h>
#include "hosted/mapping_control.hpp"
#include "hosted/sensors.hpp"
#include "hosted/uart.hpp"
#include "robots/RobotControl.hpp"

struct Input {
    uint32_t cycles;
    float pods[4], yaw, yawRate, jointYaw, jointYawRate, jointPitch, jointPitchRate;
    uint32_t refereeBytes, mode; // 0 stop, 1 simple, 2 auto; bit 2 disables auto fire
    uint8_t motors[10][8]; // yaw, pitch, index, odo, flywheel 1/2, drive 1..4
    float externalRight, externalForward, externalSpin;
    float imu[6]; // board adapter: local gyro rad/s, specific force in g
};
struct Output {
    float yaw, pitch, right, forward, spin;
    uint32_t shots, timeMs;
    int16_t motors[10];
};
static_assert(sizeof(Input) == 168 && sizeof(Output) == 48);

void feedback(tap::motor::DjiMotor& motor, const uint8_t data[8]) {
    modm::can::Message msg(motor.getMotorIdentifier(), 8);
    for (int i = 0; i < 8; ++i) msg.data[i] = data[i];
    motor.processMessage(msg);
}

int main() {
    const char* fdText = std::getenv("MCB_UART_FD");
    if (!fdText) { std::fprintf(stderr, "MCB_UART_FD is required\n"); return 2; }
    int fd = std::atoi(fdText);
    if (fcntl(fd, F_SETFL, O_NONBLOCK) < 0) return 2;
    hosted::setJetsonFd(fd);
    const char* controlFdText = std::getenv("MCB_MMM_CONTROL_FD");
    const int controlFd = controlFdText ? std::atoi(controlFdText) : -1;
    const char* debugFloodText = std::getenv("MCB_DEBUG_TEST_FLOOD");
    const bool debugFlood = debugFloodText && std::strcmp(debugFloodText, "1") == 0;
    constexpr std::array<uint8_t, 4> debugBytes{0x00, 0xa5, 0xff, 0x25};
    hosted::MappingInput mappingInput{};
    uint32_t transitions = 0;
    // Match main.cpp's static storage (zero-initialized before construction).
    static src::Drivers drivers;
    static robots::SentryControl control(&drivers);
    drivers.can.initialize();
    drivers.remote.initialize();
    drivers.refSerial.initialize();
    drivers.uart.initialize();
    drivers.recal.setIsFirstCalibrating();
    drivers.recal.setIsDoneCalibrating();
    control.initialize();
    std::fwrite("MCB3", 1, 4, stdout);
    std::fflush(stdout);
    Input in;
    auto previousLoop = std::chrono::steady_clock::now();
    while (std::fread(&in, sizeof(in), 1, stdin) == 1) {
        if (controlFd >= 0) {
            hosted::MappingInput command{};
            const auto count = recv(controlFd, &command, sizeof(command), MSG_DONTWAIT);
            if (count == sizeof(command)) {
                if (!std::isfinite(command.right) || !std::isfinite(command.forward) ||
                    !std::isfinite(command.spin)) return 2;
                mappingInput = command;
                if (command.action == 1) drivers.mapping.requestStart();
                if (command.action == 3) drivers.mapping.requestSave();
            } else if (count > 0) { return 2; }
        }
        const auto started = std::chrono::steady_clock::now();
        if (in.cycles == 0 || in.cycles > 100 || in.refereeBytes > 4096 || in.mode > 7) return 2;
        std::vector<uint8_t> ref(in.refereeBytes);
        if (!ref.empty() && std::fread(ref.data(), 1, ref.size(), stdin) != ref.size()) return 2;
        hosted::queueUart(tap::communication::serial::Uart::Uart6, ref.data(), ref.size());
        hosted::sensors.podX = in.pods[0]; hosted::sensors.podY = in.pods[1];
        hosted::sensors.podVx = in.pods[2]; hosted::sensors.podVy = in.pods[3];
        hosted::sensors.yaw = in.yaw; hosted::sensors.yawRate = in.yawRate;
        hosted::sensors.jointYaw = in.jointYaw; hosted::sensors.jointYawRate = in.jointYawRate;
        hosted::sensors.jointPitch = in.jointPitch; hosted::sensors.jointPitchRate = in.jointPitchRate;
        std::vector<uint32_t> shots;
        float periodUs = 0;
        for (uint32_t i = 0; i < in.cycles; ++i) {
            const auto loopStarted = std::chrono::steady_clock::now();
            periodUs += std::chrono::duration<float, std::micro>(loopStarted - previousLoop).count();
            previousLoop = loopStarted;
            // Match the board fixture's physical DBUS switch selection.
            const uint8_t switches = 8 | ((in.mode & 3) == 1 ? 1 : 3) << 4 |
                                      ((in.mode & 4) ? 3 : 1) << 6;
            uint8_t remote[18] = {0, 4, 32, 0, 1, switches, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4};
            const float sticks[] = {mappingInput.spin, 0, mappingInput.right, mappingInput.forward};
            uint64_t bits = static_cast<uint64_t>(switches & 0xF0) << 40;
            for (int channel = 0; channel < 4; ++channel) {
                const auto value = static_cast<uint16_t>(1024 + 660 * std::clamp(sticks[channel], -1.0f, 1.0f));
                bits |= static_cast<uint64_t>(value) << (11 * channel);
            }
            for (int byte = 0; byte < 6; ++byte) remote[byte] = bits >> (8 * byte);
            hosted::queueUart(tap::communication::serial::Uart::Uart3, remote, sizeof(remote));
            drivers.remote.read();
            // main.cpp polls I/O every 5 us between the 1 ms scheduler ticks.
            for (int poll = 0; poll < 200; ++poll) {
                drivers.refSerial.updateSerial();
                drivers.uart.updateSerial();
            }
            auto& h = control.hardware;
            unsigned motorIndex = 0;
            for (auto* motor : {&h.yawMotor, &h.pitchMotor, &h.indexMotor, &h.odoMotor,
                    &h.flywheelMotor1, &h.flywheelMotor2, &h.driveMotor1, &h.driveMotor2,
                    &h.driveMotor3, &h.driveMotor4}) feedback(*motor, in.motors[motorIndex++]);
            const float before = control.indexer.getTotalNumBallsShot();
            control.updateMapping();
            if (debugFlood) control.jetson.sendDebug(debugBytes.data(), debugBytes.size());
            drivers.commandScheduler.run();
            drivers.hitTracker.update();
            control.update();
            if (!drivers.mapping.isActive() && (in.mode & 3) != 1)
                drivers.commandScheduler.removeCommand(&control.simpleAutoDrive, true);
            if (!drivers.mapping.isActive() && (in.mode & 3) == 2 && !drivers.commandScheduler.isCommandScheduled(&control.autoDrive))
                drivers.commandScheduler.addCommand(&control.autoDrive);
            if (!drivers.mapping.isActive() && (in.mode & 3) == 3)
                control.drivetrain.setTargetTranslation(Pose2d(in.externalRight, in.externalForward, in.externalSpin), false);
            if (in.mode & 4) drivers.commandScheduler.removeCommand(&control.autoFire, true);
            if (control.indexer.getTotalNumBallsShot() > before) shots.push_back(hosted::timeMs);
            ++hosted::timeMs;
            static auto previousState = src::MappingMachine::State::Main;
            if (drivers.mapping.getState() != previousState) {
                ++transitions;
                previousState = drivers.mapping.getState();
            }
        }
        if (controlFd >= 0) {
            const float loopUs = std::chrono::duration<float, std::micro>(
                std::chrono::steady_clock::now() - started).count() / in.cycles;
            const hosted::MappingStatus status{static_cast<uint32_t>(drivers.mapping.getState()),
                hosted::timeMs, transitions, hosted::failedUartWrites(),
                drivers.mapping.readyBeep(hosted::timeMs), loopUs, periodUs / in.cycles,
                std::remainder(control.gimbal.getHostedTargetYaw() - drivers.bmi088.getYaw() +
                    control.gimbal.getYawEncoderValue(), 2 * PI)};
            if (send(controlFd, &status, sizeof(status), MSG_DONTWAIT | MSG_NOSIGNAL) != sizeof(status)) return 2;
        }
        Pose2d drive = control.drivetrain.getHostedTargetVelocity();
        Output out{control.gimbal.getHostedTargetYaw(), control.gimbal.getPrevTargetPitch(),
                   drive.getX(), drive.getY(), drive.getRotation(),
                   static_cast<uint32_t>(shots.size()), hosted::timeMs, {}};
        auto& h = control.hardware;
        unsigned motorIndex = 0;
        for (auto* motor : {&h.yawMotor, &h.pitchMotor, &h.indexMotor, &h.odoMotor,
                &h.flywheelMotor1, &h.flywheelMotor2, &h.driveMotor1, &h.driveMotor2,
                &h.driveMotor3, &h.driveMotor4}) out.motors[motorIndex++] = motor->getOutputDesired();
        std::fwrite(&out, sizeof(out), 1, stdout);
        if (!shots.empty()) std::fwrite(shots.data(), sizeof(uint32_t), shots.size(), stdout);
        std::fflush(stdout);
    }
    return std::feof(stdin) ? 0 : 2;
}
