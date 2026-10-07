// Copyright 2026 Thornbots. SPDX-License-Identifier: GPL-3.0-or-later
// Hardware fixture for the real sentry firmware. stdin/stdout are lockstep IPC;
// UART1 is a raw PTY. All protocol parsing and control run in the MCB sources.
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <vector>
#include "hosted/sensors.hpp"
#include "hosted/uart.hpp"
#include "robots/RobotControl.hpp"
#include "subsystems/odometry/OdometrySubsystemConstants.hpp"

struct Input {
    uint32_t cycles;
    float pods[4], yaw, yawRate, jointYaw, jointYawRate, jointPitch, jointPitchRate;
    uint32_t refereeBytes, mode; // 0 stop, 1 simple, 2 auto; bit 2 disables auto fire
};
struct Output {
    float yaw, pitch, right, forward, spin;
    uint32_t shots, timeMs;
};
static_assert(sizeof(Input) == 52 && sizeof(Output) == 28);

void feedback(tap::motor::DjiMotor& motor) {
    modm::can::Message msg(motor.getMotorIdentifier(), 8);
    for (auto& byte : msg.data) byte = 0;
    motor.processMessage(msg);
}

int main() {
    const char* fdText = std::getenv("MCB_UART_FD");
    if (!fdText) { std::fprintf(stderr, "MCB_UART_FD is required\n"); return 2; }
    int fd = std::atoi(fdText);
    if (fcntl(fd, F_SETFL, O_NONBLOCK) < 0) return 2;
    hosted::setJetsonFd(fd);
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
    std::fwrite("MCB1", 1, 4, stdout);
    const float start[2] = {odo::START_X, odo::START_Y};
    std::fwrite(start, sizeof(float), 2, stdout);
    std::fflush(stdout);
    Input in;
    while (std::fread(&in, sizeof(in), 1, stdin) == 1) {
        if (in.cycles == 0 || in.cycles > 100 || in.refereeBytes > 4096 || in.mode > 6 ||
                (in.mode & 3) == 3) return 2;
        std::vector<uint8_t> ref(in.refereeBytes);
        if (!ref.empty() && std::fread(ref.data(), 1, ref.size(), stdin) != ref.size()) return 2;
        hosted::queueUart(tap::communication::serial::Uart::Uart6, ref.data(), ref.size());
        hosted::sensors.podX = in.pods[0]; hosted::sensors.podY = in.pods[1];
        hosted::sensors.podVx = in.pods[2]; hosted::sensors.podVy = in.pods[3];
        hosted::sensors.yaw = in.yaw; hosted::sensors.yawRate = in.yawRate;
        hosted::sensors.jointYaw = in.jointYaw; hosted::sensors.jointYawRate = in.jointYawRate;
        hosted::sensors.jointPitch = in.jointPitch; hosted::sensors.jointPitchRate = in.jointPitchRate;
        std::vector<uint32_t> shots;
        for (uint32_t i = 0; i < in.cycles; ++i) {
            // Centred sticks, both switches up: decoded by the real DBUS driver.
            const uint8_t remote[18] = {0, 4, 32, 0, 1, 0x58, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4};
            hosted::queueUart(tap::communication::serial::Uart::Uart3, remote, sizeof(remote));
            drivers.remote.read();
            // main.cpp polls I/O every 5 us between the 1 ms scheduler ticks.
            for (int poll = 0; poll < 200; ++poll) {
                drivers.refSerial.updateSerial();
                drivers.uart.updateSerial();
            }
            auto& h = control.hardware;
            for (auto* motor : {&h.yawMotor, &h.pitchMotor, &h.indexMotor, &h.odoMotor,
                    &h.flywheelMotor1, &h.flywheelMotor2, &h.driveMotor1, &h.driveMotor2,
                    &h.driveMotor3, &h.driveMotor4}) feedback(*motor);
            const float before = control.indexer.getTotalNumBallsShot();
            drivers.commandScheduler.run();
            drivers.hitTracker.update();
            control.update();
            if ((in.mode & 3) != 1) drivers.commandScheduler.removeCommand(&control.simpleAutoDrive, true);
            if ((in.mode & 3) == 2 && !drivers.commandScheduler.isCommandScheduled(&control.autoDrive))
                drivers.commandScheduler.addCommand(&control.autoDrive);
            if (in.mode & 4) drivers.commandScheduler.removeCommand(&control.autoFire, true);
            if (control.indexer.getTotalNumBallsShot() > before) shots.push_back(hosted::timeMs);
            ++hosted::timeMs;
        }
        Pose2d drive = control.drivetrain.getHostedTargetVelocity();
        Output out{control.gimbal.getHostedTargetYaw(), control.gimbal.getPrevTargetPitch(),
                   drive.getX(), drive.getY(), drive.getRotation(),
                   static_cast<uint32_t>(shots.size()), hosted::timeMs};
        std::fwrite(&out, sizeof(out), 1, stdout);
        if (!shots.empty()) std::fwrite(shots.data(), sizeof(uint32_t), shots.size(), stdout);
        std::fflush(stdout);
    }
    return std::feof(stdin) ? 0 : 2;
}
