# MCBV3
Rose-Hulman Robomasters' (ThornBots) controls repository. Uses Taproot. Command-based and started from Teaching-Freshies

## Getting Started

First, **MAKE SURE TAPROOT IS PROPERLY INSTALLED**! If you haven't go ahead and check out our [software portal](https://thornbots-software.web.app) and 
check out our links page to get started with taproot.

Assuming you have taproot, run the following to clone this repository:

```
git clone --recursive git@github.com:Thornbots/MCBV3.git
```

If you use the Docker container, or have already cloned the repository yourself, you should instead
run:

```
git submodule update --init --recursive
```

Finally, install `pipenv` and set up the build tools:

```
pip3 install pipenv
cd MCBV3/MCB-project/
pipenv install
```

Once finished, congratulations! You're ready to start coding!

## Writing and Flashing Code

All code is written in `MCB-project/src/`. If you understand typical C++
and taproot, this should be relatively intuitive. 

To flash code, you'll need to be in the pipenv shell. If it's not running, use the following command to start it.

```
cd MCBV3/MCB-project/
pipenv shell
```

Note that this shell needs to be running to flash code, so every time you open a terminal you'll need to start it up again. Once it's running, you're ready to flash!
Don't forget, you should be in the `MCBV3/MCB-project` directory when running any of these commands! Once you're ready, run the following:

```
scons run
```

This command will build the robot code, prepping it to flash, then will proceed to flash it using `openocd`. Alternatively, you can run `scons build` to just build the code. Without `robot=`, the build warns and defaults to `ENGINEER` (see [CI](#continuous-integration)); pass `robot=` explicitly. You can specify the robot type and the system identificaton enable using the command below:

```
scons run robot=<ROBOT_TYPE> sysid=<SYSID_TYPE>
```

These arguments will work for `scons build` too!

The Jetson `CV_TARGET` wire format is documented in the bridge's
[UART protocol reference](../../ros2_dji_serial_bridge/UART_PROTOCOL.md#cv_target-id1--aim-point-and-fire-decision).
Bit 3 marks a point from Jetson patrol and is currently ignored by firmware;
bit 1 remains the separate permission for firmware patrol.

## MCB Mapping Machine (sentry)

The sentry's Drivers-owned `mapping` object exposes `requestStart()` and
`requestSave()` for operator-input integration. No switch, key or gesture is
bound. Other robot builds do not connect MMM to UART or control hooks.
Start acts only in Main and immediately enters Setup. PING 2 moves Setup or
Mapping to Ready; PING 4 moves Ready to Mapping. Save in Ready immediately
enters Done. Save in Mapping sends the incomplete-save request and stays in
Mapping. Other source states ignore these requests. Main is restored only by
restarting the firmware.

Setup and Done stop drive. Ready and Mapping use chassis-relative manual
joysticks: left horizontal/right, left vertical/forward, right horizontal/turn.
Translation magnitude is capped at 0.4 m/s and rotation at 0.5 rad/s; tune
`MAPPING_MAX_TRANSLATION_MPS` and `MAPPING_MAX_ROTATION_RADPS` in
`src/robots/sentry/MappingDrive.hpp`. No boost or autonomous drive participates.
Remote disconnect stops drive. Every active MMM state disables flywheels and
indexing, including indexer homing, and holds pitch level and yaw aligned with
the chassis front. Done adds no head-down pose. Ready lights steady green and
pulses the buzzer for 100 ms every second using wrapping millisecond timers.

`src/MappingMachine.hpp` owns state, handshake timers and counted pending
echoes without hardware dependencies. UART dispatches PING directly at receive
time, before the shared latest-message mailbox. The sentry control hook ticks
MMM before the scheduler and schedules `MappingCommand`, which persistently
owns drive, gimbal, flywheels, indexer and odometry. Sentry trigger evaluation
returns immediately throughout MMM, so ordinary triggers cannot replace these
commands. The scheduler and IMU calibration mechanism are not suspended or
re-zeroed by MMM. The hosted fixture also gates its direct drive overrides.

PING keeps id 5 and its one-byte number. Zero echoes only in Main; unknown
numbers are ignored. Every received 2 or 4 queues an echo, even from an illegal
source state; only legal state changes apply. Requests 1 and 3 retry at least
10 s after their last successful same-number send until acknowledged. Their
first matching ACK queues a final echo; duplicate ACKs may queue that final
echo again without repeating a transition. Failed sends retain pending work,
which has priority over periodic POSE/REF_SYS telemetry. Legal save API calls
always apply their state transition immediately. Additional save transmissions
queue behind the preceding ACK/final leg and its 10 s same-number quiet
interval to prevent overlapping indistinguishable requests. ACK means receipt,
not durable storage; Done
can precede completion of Jetson's asynchronous save.

Hosted GoogleTests cover the transition matrix, duplicate/failing UART traffic,
retry timing and rollover, joystick caps and sentry restrictions. The hosted
test runner links the actual firmware objects built by `build-sim`.

## Automatic indexer recovery

Automatic aiming returns its indexer to idle when referee state allows shooting,
and stops it while shooting is forbidden. This releases the stop flag left by
calibration or pre-game SETUP; previously the firmware could request a shot
while its physical indexer stayed stopped. Heat, power, projectile-presence and
shot-spacing checks still decide whether a request is allowed. Manual aiming
does not take ownership of the indexer.

The simulator's compiled-firmware regression checks both a requested shot and
nonzero CAN motor output after SETUP becomes IN_GAME. A hosted shot counter
alone is not evidence that the physical indexer moved; board execution and
modeled exits are documented at [sim](../../sim/README.md#type-c-board-execution-with-renode).

## Continuous integration

GitHub Actions compiles ARM firmware for `infantry`, `hero`, and `sentry`
on `main` pushes and pull requests into `main`, including all four system identification modes.
`nightly` runs no CI on pushes or pull requests: run the checks in
[CI](../../docs/CI.md#local-checks-for-prs-into-nightly) locally first, and the
[nightly tests](../../docs/CI.md#nightly-tests) cover its hosted firmware build. Each build uploads its ELF file for
inspection; CI never flashes a board. The hosted sentry job runs the package's
geometry and chassis GoogleTests, then builds real firmware and exercises its
UART parser, referee gates, aiming, firing, and driving through the pinned
current C++ `Thornbots/sim` fixture. A missing executable, empty test report,
or skipped UART test fails CI.

Run the C++ tests in a Linux terminal with GCC 14 and GoogleTest installed:

```bash
CXX=g++-14 tools/test_cpp.sh
(cd MCB-project && scons build-sim robot=sentry profile=release compiler-suffix=-14 -j4)
CXX=g++-14 tools/test_hosted_cpp.sh
```

This runner tests package-owned code directly. The historical `scons run-tests`
target also compiles vendored Taproot test suites and needs additional GoogleMock
dependencies. The UART fixture revision is pinned in
`.github/workflows/firmware.yml`; update it deliberately when the wire protocol
changes. ARM builds use Ubuntu 24.04's GCC 13 cross compiler; hosted builds use GCC 14.
All builds treat compiler warnings as errors. Hosted builds use the system GCC
by default; `compiler-suffix=-14` selects GCC 14 explicitly.

The `engineer` target (formerly `oldinfantry` / `oldstandard`) is excluded:
its indexer homing offset and power limiter torque scaling are missing. Restore measured calibration values before
enabling that target in CI. Build selection now uses `robot=engineer` (or
`ENGINEER`); the old target names are rejected. Its hardware lives in
`src/robots/engineer/EngineerHardware.hpp` and it retains the shared infantry
controls and legacy tuning pending calibration. An omitted `robot` still selects
this target, so specify `robot=sentry`, `robot=infantry`, or `robot=hero`
for a calibrated build.

`main` preserves its regenerated Taproot IMU API in radians and radians per
second. The hosted fixture uses the same units. Its referee schema uses the
2025 Taproot RFID fields: resupply outside/inside exchange at bits 19/20 and
central buff at bit 23. The C++ fixture encodes this current layout directly;
these checks do not measure robot calibration or field RFID placement.

## Aim and navigation odometry

Sentry POSE `odom_status` is 0 (`ODOM_PODS`) while the last successful Pico
I2C read is less than 500 ms old, and 2 (`ODOM_I2C_DEAD`) before the first
success or at age >=500 ms. On the sentry the Pico values update about every
260 ms (2026-10-26), so 50 ms flagged healthy pods stale 80% of the time. The
next transmitted POSE carries the change;
this flag does not alter pod polling, bus recovery, or odometry values.
Successful reads containing frozen values still report 0.

`OdometrySubsystem::getAimX/getAimY` expose field-aligned raw pod odometry.
POSE and `AutoAimAndFireCommand` use these accessors. `getX/getY` add the
navigation offset and remain the input to navigation controllers.
RELOCALIZE replaces only that offset; raw POSE and held CV_TARGET aim remain
continuous as the pods move. Boot/team selection can reanchor raw aim: a red
to blue start changes the field origin by about 9.25 m and changes startup
heading. Frame conventions and payload semantics are owned by the workspace's
[UART protocol](../../ros2_dji_serial_bridge/UART_PROTOCOL.md#message-types).
Deploy matching firmware and Jetson packages together: unchanged payload
sizes cannot detect a frame-semantic mismatch.

The sentry drivetrain matches main, whose hold is confirmed on the real robot:
the default stop command cuts wheel current, and centered manual drive keeps a
zero wheel-velocity setpoint through the velocity controller. Neither latches
a pod position.

Board startup preserves the three-second remote power-on guard using the
already initialized SysTick clock and `WFI` between interrupts. The former
DWT busy wait spent the whole guard repeatedly reading a peripheral register
in Renode. The following remote-disconnected guard and IMU calibration still
execute normally; no simulator hook skips firmware initialization.

The wire fire delay includes indexer latency and an additional alignment wait.
`AutoAimAndFireCommand` clamps the indexing wait to zero below its calibrated sentry
latency estimate. Jetson must include that estimate when sending a scheduled
exit. The hosted chamber-exit replay measured 94 ms; hardware
indexing and UART delay still need measurement.

Absolute aim points use a filtered angular-rate estimate over the 1 ms
controller ticks: a 40 Hz point update must not become a one-tick velocity
impulse. The estimator wraps yaw differences, treats jumps above 0.25 rad as
target handoffs, and resets when leaving absolute aiming. Manual incremental
commands retain their rate semantics.

The gimbal controller uses the main scheduler's 1 ms interval. Its 8 ms yaw
predictor integrates geared output torque and the configured viscous/static
friction; the torque history starts at zero. Simulator sensor age must match
this predictor. The sentry ballistic solver assumes 24 m/s; this is a model
constant, not a new physical measurement. Calibration evidence lives in the
[sim accuracy investigation](../../sim/docs/aim-accuracy-2026-10-12.md).

## GCC compatibility

### Hosted hardware fixture

The hosted `MCB3` hardware pipe carries ten DJI CAN feedback packets and
returns raw motor command words. Gimbal encoders and velocities use the same
Taproot decoder as the board; only the hosted yaw home is fixed to the sim
joint's boot zero. Sim applies motor forces inside gz's 1 ms step; its
[MCB emulator](../../sim/README.md#mcb-emulator) owns the plant and its limits.
The hosted remote switches follow stop/simple drive and automatic aim/fire
selections, including arming after startup. The private pipe also carries
three-axis gyro/specific force for the board adapter; the native fixture keeps
its direct sensor interfaces. External velocity and NAV_GOAL drive remain
explicit hosted fixture modes. E2E stages select the firmware waypoint command.
The Jetson UART format is unchanged. Hosted execution
still bypasses board boot/calibration and cannot establish STM32 timing.
Sim's optional [Type C board backend](../../sim/README.md#type-c-board-execution-with-renode)
executes the normal hardware release ELF through its peripheral drivers;
its maintained model limits and build instructions live there.

Motor feedback uses Taproot's encoder interface: positions and velocities are
in radians and radians per second. Flywheel and legacy indexer PID inputs are
explicitly converted to RPM. UART timeouts use the boot millisecond clock and
unsigned subtraction so they survive its 32-bit rollover.

The firmware provides no POSIX files, process control, or wired entropy source.
Gimbal and odometry system-identification generators are constructed in place
with `std::mt19937::default_seed`. Their sequences are reproducible; normal
robot builds do not consume them. Assigning a temporary generator in these
global constructors exceeded the board's 3 KiB startup stack during the
Type C emulator boot investigation. No control gains, offsets, or command
selection change with this initialization fix.
Package-owned newlib hooks return failure with `ENOSYS`; `_getpid` identifies
the single firmware process. Hardware I/O still goes through Taproot.
These hooks let modern newlib link without its warning-bearing nosys fallbacks.

The generated Taproot/modm copies checked into this repository contain local
compatibility fixes: explicit iterator traits instead of `std::iterator`,
`delay_us`, guarded peripheral clock writes, and erase/remove for null command
mappings. Preserve these fixes when regenerating from the pinned upstream
submodules. The hosted C++ regression tests check iterator traits, null command
filtering, UART payload bounds, and clock rollover.
