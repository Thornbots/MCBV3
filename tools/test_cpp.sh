#!/usr/bin/env bash
# Exercise package-owned geometry and chassis code without vendor test suites.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../MCB-project"
python3 ../tools/test_mapping_hook.py
mkdir -p build/unit-tests
"${CXX:-g++}" -std=c++20 -Wall -Wextra -Werror -DSENTRY -Isrc \
    test/PoseTest.cpp test/ChassisControllerTest.cpp test/NewlibSyscallsTest.cpp test/YawControllerTest.cpp test/HitTrackerTest.cpp test/MappingMachineTest.cpp \
    src/platform/newlib_syscalls.cpp src/subsystems/drivetrain/ChassisController.cpp \
    src/subsystems/gimbal/controllers/YawController.cpp \
    -lgtest_main -lgtest -pthread -o build/unit-tests/mcb-tests
build/unit-tests/mcb-tests "$@"
