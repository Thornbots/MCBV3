# MCBV3

Firmware is an opt-in workspace submodule. Hosted builds use fake hardware;
ARM builds target the real board. Humble branches remain frozen.

## CI

GitHub CI runs on pushes and PRs outside frozen Humble branches. Shared lint
is pinned to workspace `7e6fdb673f7b`. Existing diagnostics are recorded in
`.github/quality-baseline.json`; new diagnostics fail. Do not expand the
baseline to hide regressions. Syntax errors always fail.
