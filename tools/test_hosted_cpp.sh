#!/usr/bin/env bash
# Test UART timing/bounds and vendored compatibility fixes against hosted libraries.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../MCB-project"
mkdir -p build/unit-tests
"${CXX:-g++}" -std=c++20 -Wall -Wextra -Werror -funsigned-char -funsigned-bitfields \
    -DSENTRY -DPLATFORM_HOSTED -DMCB_HOSTED \
    -Isrc -Itaproot/src -Itaproot/sim-modm/hosted-linux/modm/src \
    -Itaproot/modm/ext/cmsis/dsp -Itaproot/modm/ext/cmsis/core \
    test/FirmwareRegressionTest.cpp src/communication/UARTCommunication.cpp src/hosted/uart.cpp \
    build/sim/scons-release/libtaproot.a build/sim/scons-release/modm/libmodm.a \
    -lgtest_main -lgtest -pthread -o build/unit-tests/firmware-regression-tests
build/unit-tests/firmware-regression-tests "$@"
