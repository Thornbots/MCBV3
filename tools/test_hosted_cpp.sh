#!/usr/bin/env bash
# Test UART timing/bounds and vendored compatibility fixes against hosted libraries.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../MCB-project"
mkdir -p build/unit-tests
"${CXX:-g++}" -std=c++20 -Wall -Wextra -Werror -funsigned-char -funsigned-bitfields \
    -DSENTRY -DPLATFORM_HOSTED -DMCB_HOSTED \
    -Isrc -Itaproot/src -Itaproot/sim-modm/hosted-linux/modm/src \
    -Itaproot/modm/ext/cmsis/dsp -Itaproot/modm/ext/cmsis/core \
    test/FirmwareRegressionTest.cpp test/DrivetrainStopCommandTest.cpp \
    test/SimpleAutoDriveCommandTest.cpp \
    src/subsystems/drivetrain/SimpleAutoDriveCommand.cpp \
    src/subsystems/drivetrain/MoveToPositionCommand.cpp \
    src/communication/UARTCommunication.cpp src/hosted/uart.cpp \
    src/subsystems/drivetrain/DrivetrainSubsystem.cpp src/subsystems/drivetrain/ChassisController.cpp \
    src/subsystems/gimbal/GimbalSubsystem.cpp \
    src/subsystems/gimbal/controllers/YawController.cpp src/subsystems/gimbal/controllers/PitchController.cpp \
    src/subsystems/odometry/OdometrySubsystem.cpp \
    src/subsystems/odometry/controllers/OdoController.cpp \
    build/sim/scons-release/libtaproot.a build/sim/scons-release/modm/libmodm.a \
    -lgtest_main -lgtest -pthread -o build/unit-tests/firmware-regression-tests
build/unit-tests/firmware-regression-tests "$@"
