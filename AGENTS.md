# MCBV3

Firmware is an opt-in workspace submodule. Hosted builds use fake hardware;
ARM builds target the real board. Humble branches remain frozen.

## CI

GitHub CI runs on PRs and main pushes; manual runs are available. Shared lint
is pinned to workspace `7e6fdb673f7b`. Existing diagnostics are recorded in
`.github/quality-baseline.json`; new diagnostics fail. Do not expand the
baseline to hide regressions. Syntax errors always fail.

## Builds

Use GCC 14 for hosted builds (`compiler-suffix=-14`, `CXX=g++-14`) and
Ubuntu 24.04 ARM GCC 13. Warnings are errors; fix sources, not flags.
Preserve the generated-vendor compatibility patches described in README.md.

Engineer replaces oldinfantry/oldstandard and stays out of CI until its
indexer homing offset and drivetrain torque scaling are calibrated.
