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

## On-demand firmware data

Use `drivers->uart.mcbLog(channel, ptr, length)` from the main scheduler context;
it copies the bytes into a bounded queue and returns false on rejection. The
`mcbLog(channel, value)` helper accepts trivially copyable structs that fit.
Define serialization explicitly: native struct padding/endianness are part of
what a raw struct sends, so packed structs or explicit byte encoding are preferable.
`drivers->uart.mcbText("value=%u", value)` is the bounded text convenience,
replacing the unused broken `PRINT` macro. Neither API is ISR/thread safe.
`droppedData()` exposes rejected records locally, with transport reports also
sent on the reserved channel.

The [wire contract](../../ros2_dji_serial_bridge/UART_PROTOCOL.md#mcb_data-id6--firmware-owned-opaque-data)
owns channel reservations, bounds, scheduling and stamps. The
[firmware data workflow](../../ros2_dji_serial_bridge/README.md#firmware-data-over-wifi)
explains live Foxglove viewing and copying/decoding recorded bags over WiFi.

### Odometry I2C diagnostics

On `sentry`, `LocalizationPico` logs each I2C read of the Pico (`0x55`, one
16-byte reply) on two channels. All fields are little-endian, packed, no padding.
`result` is `modm::I2cMaster::Error`, latched when the transaction detaches:
0 NoError, 1 SoftwareReset (bus reinit), 2 AddressNack, 3 DataNack,
4 ArbitrationLost, 5 BusCondition, 6 BusBusy (includes stuck-bus timeouts), 7 Unknown.
modm has no transaction timeout: a read that never completes produces no
records, only a growing `ms_since_attempt` in the summary.

Channel 16, one record per attempt, 36 bytes:

| Off | Type | Field | Meaning |
| --- | --- | --- | --- |
| 0 | u32 | `time_us` | MCB clock when the attempt completed |
| 4 | u32 | `seq` | Attempt number since boot, from 1; gaps are unlogged attempts |
| 8 | u8 | `result` | Error code above |
| 9 | u8 | `flags` | bit 0 success (held values updated); bit 1 bus reinit before this attempt |
| 10 | u16 | `pending` | Failure records waiting on the MCB when this one was made |
| 12 | 4×f32 | `enc1_pos, enc2_pos, enc1_speed, enc2_speed` | Receive buffer after the attempt; on failure, stale or partial, not used |
| 28 | f32 | `held_x` | `getRawX()` after the attempt |
| 32 | f32 | `held_y` | `getRawY()` after the attempt (field forward) |

Channel 17, about 1 Hz summary, 48 bytes:

| Off | Type | Field | Meaning |
| --- | --- | --- | --- |
| 0 | u32 | `time_us` | MCB clock when queued |
| 4 | u32 | `last_seq` | Last attempt number |
| 8 | 8×u16 | `results[0..7]` | Attempts per result code since the previous summary, saturating |
| 24 | u32 | `lost_failures` | Failure records not logged since boot (16-entry buffer full) |
| 28 | u32 | `decimated_successes` | Success records replaced before sending, since boot |
| 32 | u32 | `fifo_drops` | `droppedData()` for all channels since boot |
| 36 | u32 | `ms_since_success` | `UINT32_MAX` if never |
| 40 | u32 | `ms_since_attempt` | Since the last completed attempt; `UINT32_MAX` if never |
| 44 | u16 | `bus_reinits` | `I2CCommunication` bus resets since boot |
| 46 | u16 | reserved | 0 |

The Pico is polled as fast as the bus allows (about 1–2 kHz); the link fits
about one 46-byte data frame per 10 ms POSE cycle. Records wait in the logger,
not the MCB data FIFO: summaries first, then failures, then the latest success
at most every 20 ms while a FIFO slot stays spare. Successes are decimated;
failures are only lost if more than 16 wait, and are always counted.
POSE/REF_SYS timing is unchanged (`OdomI2cLogTest`). The hosted firmware
fixture emits records only when `MCB_ODOM_I2C_LOG` is set.

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
