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
    const float start[2] = {odo::START_X, odo::START_Y};
    std::fwrite(start, sizeof(float), 2, stdout);
    std::fflush(stdout);
    const bool dataTestLoad = std::getenv("MCB_DATA_TEST_LOAD") != nullptr;
    Input in;
    while (std::fread(&in, sizeof(in), 1, stdin) == 1) {
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
        for (uint32_t i = 0; i < in.cycles; ++i) {
            // Match the board fixture's physical DBUS switch selection.
            const uint8_t switches = 8 | ((in.mode & 3) == 1 ? 1 : 3) << 4 |
                                      ((in.mode & 4) ? 3 : 1) << 6;
            const uint8_t remote[18] = {0, 4, 32, 0, 1, switches, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4};
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
            if (dataTestLoad) {
                const uint8_t sample[] = {0x00, 0xA5, 0xFF, 0x2A};
                drivers.uart.mcbLog(42, sample, sizeof(sample));
            }
            drivers.commandScheduler.run();
            drivers.hitTracker.update();
            control.update();
            if ((in.mode & 3) != 1) drivers.commandScheduler.removeCommand(&control.simpleAutoDrive, true);
            if ((in.mode & 3) == 2 && !drivers.commandScheduler.isCommandScheduled(&control.autoDrive))
                drivers.commandScheduler.addCommand(&control.autoDrive);
            if ((in.mode & 3) == 3)
                control.drivetrain.setTargetTranslation(Pose2d(in.externalRight, in.externalForward, in.externalSpin), false);
            if (in.mode & 4) drivers.commandScheduler.removeCommand(&control.autoFire, true);
            if (control.indexer.getTotalNumBallsShot() > before) shots.push_back(hosted::timeMs);
            ++hosted::timeMs;
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
