#!/usr/bin/env bash
# Test UART timing/bounds and vendored compatibility fixes against hosted libraries.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../MCB-project"
mkdir -p build/unit-tests
shopt -s globstar nullglob
objects=()
for object in build/sim/scons-release/src/**/*.o; do
    case "$object" in
        */src/tap/*|*/src/hosted/main.o) continue ;;
    esac
    objects+=("$object")
done
"${CXX:-g++}" -std=c++20 -Wall -Wextra -Werror -funsigned-char -funsigned-bitfields \
    -DSENTRY -DPLATFORM_HOSTED -DMCB_HOSTED \
    -Isrc -Itaproot/src -Itaproot/sim-modm/hosted-linux/modm/src \
    -Itaproot/modm/src \
    -Itaproot/modm/ext/cmsis/dsp -Itaproot/modm/ext/cmsis/core \
    test/LocalizationPicoTest.cpp test/FirmwareRegressionTest.cpp test/DrivetrainStopCommandTest.cpp \
    test/SimpleAutoDriveCommandTest.cpp test/MappingControlTest.cpp "${objects[@]}" \
    build/sim/scons-release/libtaproot.a build/sim/scons-release/modm/libmodm.a \
    -Wl,--wrap=_ZN3tap4gpio4Leds3setENS1_6LedPinEb \
    -Wl,--wrap=_ZN3tap6buzzer8playNoteEPNS_4gpio3PwmEj \
    -Wl,--wrap=_ZN3tap6buzzer13silenceBuzzerEPNS_4gpio3PwmE \
    -lgtest_main -lgtest -pthread -o build/unit-tests/firmware-regression-tests
build/unit-tests/firmware-regression-tests "$@"
