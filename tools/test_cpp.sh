#!/usr/bin/env bash
# Exercise package-owned geometry and chassis code without vendor test suites.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../MCB-project"
mkdir -p build/unit-tests
"${CXX:-g++-11}" -std=c++17 -DSENTRY -Isrc \
    test/PoseTest.cpp test/ChassisControllerTest.cpp \
    src/subsystems/drivetrain/ChassisController.cpp \
    -lgtest_main -lgtest -pthread -o build/unit-tests/mcb-tests
build/unit-tests/mcb-tests "$@"
