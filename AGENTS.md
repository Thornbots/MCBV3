# MCBV3

Follow [workspace rules](../../AGENTS.md) and [CI](../../docs/CI.md).
This is an opt-in submodule. Read [build/test guidance](README.md#continuous-integration)
and [GCC compatibility](README.md#gcc-compatibility) before firmware changes.

- Hosted builds use fake hardware, GCC 14; board builds use ARM GCC 13.
- Warnings are errors: fix sources, not flags; preserve generated-vendor patches.
- Keep `engineer` out of CI until homing offset and drivetrain torque scaling
  are calibrated. Always select the intended robot explicitly.
