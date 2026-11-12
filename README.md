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

In the VS Code dev container (Linux host), `Ctrl+Shift+B` runs `scons run` for the
robot selected as the C/C++ configuration in the status bar. The container matches CI's
toolchain and sees the host's USB debug probe; plug the probe in before flashing.
Open it from the Isaac ROS workspace's `firmware/MCBV3` submodule: it mounts the whole
workspace at its host path, so git and the workspace policy hooks work. Standalone clones
are not supported.

The Jetson `CV_TARGET` (idle look point) and `TARGET_STATE` (tracked armor model)
wire formats are in the bridge's
[UART protocol reference](../../ros2_dji_serial_bridge/UART_PROTOCOL.md#cv_target-id1--idle-look-point);
the aim planner is under [Aim and navigation odometry](#mcbv3-aiming-planner).

## Opaque debug blobs

`JetsonSubsystem::sendDebug(const uint8_t* data, size_t length)` copies a raw
0–128 byte value for MCB→Jetson message id 6. There is no content schema.
It returns false for oversize input, a null pointer with nonzero length, or
an occupied single pending slot; accepted values are retained until attempted.
A null pointer with zero length sends an empty payload. Call this cooperative
main-loop API per value at any rate; it allocates nothing and never writes or
waits for UART. It is not an ISR or concurrent-thread API. There is no producer
timer, growing queue, or delivery guarantee.

`refresh()` services RX and MMM PING before telemetry and DEBUG. Mandatory
telemetry under continuous service keeps its original 10 ms deadline sequence
(nine POSE then one REF_SYS), sending overdue slots before another DEBUG.
After a scheduler service gap longer than 100 ms, including boot calibration,
the cycle restarts with its first POSE deadline 10 ms after service resumes.
The single pending slot
is drained only when occupied; there is no fixed throttle. Failed/partial debug
writes discard that value and still consume their full airtime budget.

UART initialization and wire-time accounting share `DJISerial::UART_BAUD_RATE`
(115200, 8N1). Wire-idle requires full airtime since enqueue, buffer-empty, and the hardware
transmission-complete flag (hosted models airtime). Admission reserves DEBUG
airtime, maximum telemetry airtime and 2 ms for servicing both completions; both
frames must finish before the second future telemetry deadline.
The [wire contract](../../ros2_dji_serial_bridge/UART_PROTOCOL.md#debug-id6--opaque-firmware-debug)
owns latency bounds, scheduler assumptions and flood guarantees. A baud too
slow for that bound simply leaves oversized pending debug unsent. Telemetry
samples current state when sent; RX continues independently of TX budget.

The hosted-only executable accepts `MCB_DEBUG_TEST_FLOOD=1` alongside the usual
`MCB_UART_FD` and binary stdin/stdout step fixture. Every simulated 1 ms main
step calls `sendDebug` with 128 known bytes: `00 a5 ff 25` followed by `0x04` through `0x7f`. All other
values of the environment variable leave it off. Run
`MCB-project/build/sim/scons-release/MCB-project.elf` with the existing sim
fixture transport; it is not a standalone interactive executable. The hardware
source graph excludes `src/hosted`, so this environment hook cannot enter board
firmware. Hosted regression tests flood the real subsystem and UART parser,
check exact mandatory slots and CV_TARGET/RELOCALIZE receipt, and model physical
wire drain despite the hosted UART's always-true buffer-empty flag.

### Optional CAN bus health (layout v1)

Build with `scons build robot=sentry can-bus-health=true` (also supported by
`build-sim`, `infantry` and `hero`). The default is `can-bus-health=false`:
normal/match builds contain no counters, feedback wrapper or telemetry producer.
Enable this explicitly only for diagnostic builds. No Taproot sources change.
After an enabled GCC 14 hosted build, run
`MCB_CAN_BUS_HEALTH=1 CXX=g++-14 tools/test_hosted_cpp.sh` so the test runner
uses the same class layout and checks opt-in DEBUG output. Omit that environment
setting after a default build.

Each approximately one-second cooperative window queues one 103-byte DEBUG
payload through `sendDebug`; an occupied slot drops the window without retry.
Sequence advances even for dropped windows. Reporting starts after control
initialization, retains actual elapsed time across service gaps, and resets
all counts each window. Existing DEBUG admission, RX and mandatory telemetry
scheduling remain in charge; calibration may defer transmission.

All integers are unsigned little-endian, with no C struct padding:

| Offset | Bytes | Meaning |
| --- | --- | --- |
| 0 | 1 | Layout version, 1 |
| 1 | 4 | Window sequence, starts at 0, wraps modulo 2³² |
| 5 | 4 | Actual window length in milliseconds (wrapping boot-clock difference) |
| 9 | 4 | Number of sampled 1 ms command cycles |
| 13 | 45 | CAN1 record below |
| 58 | 45 | CAN2 record below |

Bus record offsets are relative to its start:

| Offset | Bytes | Meaning |
| --- | --- | --- |
| 0 | 4 | Cycles with `isReadyToSend(bus) == false` |
| 4 | 4 | `sendMessage` failures; zero with validity bit 1 clear (unavailable) |
| 8 | 32 | Eight uint32 feedback counts, motor IDs 1–8 (0x201–0x208) |
| 40 | 4 | bxCAN ESR snapshot, read once at window close |
| 44 | 1 | Validity flags: bit 0 ESR available, bit 1 send failures available; others zero |

ESR fields follow STM32F407 bxCAN: EWGF bit 0, EPVF bit 1, BOFF bit 2,
LEC bits 4–6, TEC bits 16–23, REC bits 24–31. Hosted/sim reports zero ESR
with bit 0 clear. No register is written or cleared. LEC is the last-error
code, not an error count; see ST's [RM0090](https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf).
Taproot does not expose per-bus `sendMessage` results, so bit 1 stays clear.

Readiness is sampled immediately before `encodeAndSendCanData`; a mailbox can
drain before Taproot's own later check, so this is a skipped-command indicator,
not an exact count of its internal branch or measured bus utilization. Counts
cover firmware-dispatched feedback for configured motors, not every wire frame
or frames lost before dispatch. Zero feedback can mean an unconfigured ID or
a missing motor; compare with that robot's hardware assignments. The usual
expectation is 1000 feedback frames/s per configured motor. CAN1 yaw (ID7,
0x207) has lower arbitration priority than the sentry's other CAN1 feedback;
compare its rate and ESR with the other IDs when investigating saturation.

Decode a bag in a ROS terminal with the bridge messages installed, or decode
payload-only hex with standard Python (no ROS dependencies):

```bash
python3 tools/decode_can_health.py --bag /path/to/bag
python3 tools/decode_can_health.py --hex '01 ...'
```

The bag reader defaults to `/dji_serial_bridge/debug` (`--topic` overrides it)
and skips unrelated DEBUG layouts. Output includes not-ready percentage of
sampled cycles, events/s, each ID's feedback rate versus 1000/s, and ESR fields.
These are diagnostic indicators; they do not infer a precise bus-load percentage.

`--command-frames CAN1,CAN2` (command frames each bus sends per 1 ms cycle,
e.g. `2,1`; one `0x200`/`0x1FF`/`0x1FE`/`0x2FF` frame commands up to four
motors) adds an estimated load range per bus: counted feedback plus commands on
ready cycles, times 111–135 bits per 8-byte standard frame (unstuffed to
worst-case stuffing) at 1 Mbit/s. It is a lower bound: other nodes, error
frames, retransmissions and feedback IDs above 8 are not counted. Saturation
shows as rising not-ready with the highest IDs starving first and a clean ESR;
rising TEC/REC or a nonzero LEC indicates an electrical fault instead. A
SocketCAN `canbusload` tap measures actual load.

## MCB Mapping Machine (sentry)

The sentry's Drivers-owned `mapping` object exposes `requestStart()` and
`requestSave()`. With the left switch down (offset joystick look), holding the
right stick up past `PITCH_MAPPING_THRESHOLD` for `MAPPING_HOLD_TIME` (both in
`GimbalSubsystemConstants.hpp`) starts mapping. Inside MMM the same hold, after
a release, requests one save per hold; holds outside Ready and Mapping are
discarded. No input leaves MMM. Other robot builds do not connect MMM to UART or
control hooks.
Start acts only in Main and immediately enters Setup. PING 2 moves Setup or
Mapping to Ready; PING 4 moves Ready or Done to Mapping. PING 6 (fail) moves
Setup to Done and cancels start retries. Save in Ready immediately
enters Done. Save in Mapping sends the incomplete-save request and stays in
Mapping. Other source states ignore these requests. Main is restored only by
restarting the firmware. A failed final save can return Done to Mapping with
PING 4, restoring manual drive under Mapping's restrictions. This cancels any
pending save request, queued save or final save echo; only a fresh operator save
can send another save request.

Setup and Done stop drive. Ready and Mapping use chassis-relative manual
joysticks: left horizontal/right, left vertical/forward, right horizontal/turn.
Translation magnitude is capped at 0.4 m/s and rotation at 0.5 rad/s; tune
`MAPPING_MAX_TRANSLATION_MPS` and `MAPPING_MAX_ROTATION_RADPS` in
`src/subsystems/jetson/MappingDrive.hpp`. No boost or autonomous drive participates.
Remote disconnect stops drive but keeps odometry pod pointing active in every
MMM state. Every active MMM state disables flywheels and
indexing, including indexer homing, and holds pitch level and yaw aligned with
the chassis front. Done adds no head-down pose. Ready lights steady green and
pulses the buzzer for 100 ms every second using wrapping millisecond timers.

`src/util/MappingMachine.hpp` owns state, handshake timers and counted pending
echoes without hardware dependencies. UART dispatches PING directly at receive
time, before the shared latest-message mailbox. `JetsonSubsystem::refresh()`
services MMM pings before POSE/REF_SYS; the sentry control hook calls the same
ping service while IMU calibration skips scheduler refreshes. The hook schedules
`MappingCommand`, which persistently owns drive, gimbal, flywheels, indexer and
odometry, including disconnected output restrictions and Ready LEDs/buzzer.
Ending the command clears green and silences the buzzer. Sentry trigger evaluation
returns immediately throughout MMM, so ordinary triggers cannot replace these
commands. The scheduler and IMU calibration mechanism are not suspended or
re-zeroed by MMM. The hosted fixture also gates its direct drive overrides.

PING keeps id 5 and its one-byte number. Zero echoes only in Main; unknown
numbers are ignored. Every received 2, 4 or 6 queues an echo, even from an illegal
source state; only legal state changes apply. Requests 1 and 3 retry at least
0.5 s after their last successful same-number send until acknowledged. Their
first matching ACK queues one final echo; all later echoes are ignored,
including the Jetson's ACK of that final leg. Distinct saves can be sent
inside five seconds: the Jetson ACKs them but suppresses work until there
has been a five-second same-number receive gap. Failed sends retain pending work,
which has priority over periodic POSE/REF_SYS telemetry. Legal save API calls
always apply their state transition immediately. Additional save transmissions
queue behind the preceding ACK/final leg and its 0.5 s send interval
to prevent overlapping indistinguishable requests. ACK means receipt,
not durable storage; Done
can precede completion of Jetson's asynchronous save.

Hosted GoogleTests cover the transition matrix, duplicate/failing UART traffic,
retry timing and rollover, joystick caps and sentry restrictions. The hosted
test runner links the actual firmware objects built by `build-sim`.

The hosted executable optionally accepts `MCB_MMM_CONTROL_FD`, a private
nonblocking `SOCK_SEQPACKET` fixture channel defined in
`src/hosted/mapping_control.hpp`. It invokes the same start/save APIs and
supplies DBUS joystick channels, returning state, Ready beep timing, UART write
failures and measured execution time per simulated 1 ms loop. It changes no
UART payload or board input binding. The hardware source graph excludes
`hosted/`, and `tools/test_cpp.sh` verifies both that exclusion and the empty
control header without `MCB_HOSTED`. The [sim MMM suite](../../sim/README.md#run-the-tests)
exercises the compiled executable against the real Jetson node and save service.

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
continuous as the pods move. Raw aim boots at `(0, 0)` for either team; pod
increments use the fixed `π/2` chassis boot heading for both teams. Team
selection never moves the accumulated raw position, rotates pod axes or
clears the navigation offset. NAV_GOAL driving stops the drivetrain until the first
finite RELOCALIZE establishes the field navigation offset. Invalid non-finite
coordinates leave the gate and previous offset unchanged. NAV_GOALs received
before localization are discarded; driving needs a fresh goal after the fix.

POSE adds this boot heading to the sampled IMU-relative gimbal yaw; aiming and driving
remove it when converting field targets into local commands. This also applies
to robots without odometry. Firmware owns no route or team start XY: the
Jetson's [waypoint navigation](../../nav_pkg/README.md) sends NAV_GOALs in the
field frame, and the right switch's up position schedules `AutoDriveCommand` to
follow them. It holds the nearest field quarter turn while travelling, because
the 0.84 m spin sweep is wider than the Starting Zone's 0.745 m doorway, and
spins only within `SPIN_START_RADIUS` of the goal (with hysteresis); during
the 3v3 countdown it spins in place.
Frame conventions and payload semantics are owned by the workspace's
[UART protocol](../../ros2_dji_serial_bridge/UART_PROTOCOL.md#message-types).
Deploy matching firmware and Jetson packages together: unchanged payload
sizes cannot detect a frame-semantic mismatch.

`AutoAimAndFireCommand` turns the barrel's line, not the yaw axis, through
the CV_TARGET point: it adds `asin(OFFSET_X_PITCH_PIVOT_TO_BARREL / range)` to
the bearing. The sentry's -1.9 mm is the bore centre in the team's
2026-10-26 low-resolution CAD (the older export put it 12.8 mm left).

The sentry drivetrain matches main, whose hold is confirmed on the real robot:
the default stop command cuts wheel current, and centered manual drive keeps a
zero wheel-velocity setpoint through the velocity controller. Neither latches
a pod position.

Board startup preserves the three-second remote power-on guard using the
already initialized SysTick clock and `WFI` between interrupts. The former
DWT busy wait spent the whole guard repeatedly reading a peripheral register
in Renode. The following remote-disconnected guard and IMU calibration still
execute normally; no simulator hook skips firmware initialization.

### MCBV3 aiming planner

The MCB owns aiming after tracking
([TARGET_STATE](../../ros2_dji_serial_bridge/UART_PROTOCOL.md#target_state-id7--tracked-enemy-armor-model)):
`src/subsystems/jetson/AimPlanner.{hpp,cpp}` is the Jetson's former `point_to_cv_target`
planner ported to float with no hardware dependencies (hosted `AimPlannerTest`).
Semantics are the Jetson's: `choose_panel` with held-panel hysteresis, `plan_shot`
(panel lead; spin-mode shotgating, or chase when enabled; acceleration and shooter-velocity
lead on a time-of-flight fixed point), `fire_exit_s`, spin hysteresis and the fire lock.
Rationale and measurements: [thornbots_pkg design notes](../../thornbots_pkg/README.md).

`AutoAimAndFireCommand` priority: held target state (state age under the 0.5 s timeout)
over the idle CV_TARGET point (200 ms validity) over hit turn and firmware patrol.
A held state makes no hit turn, and the idle point never fires. Age = wire age plus time
since receipt. Every replan period the command propagates the state, plans from the
shooter at the state's stamp (odometry position less velocity times state age, at the pivot
height) and holds that aim until the next period. `valid` clear aims at `panel` without
lead or fire. Fire needs `valid`, confidence of at least 0.5, detection age at most
0.15 s, the fire lock, and a planned delay. A request waits
`max(0, alignment + max(latency, min delay) - FIRING_LATENCY_TIME)` from the plan time
(the exit time includes the indexer latency, which the sentry timed at 20 ms on the
robot 2026-10-28). While a request is pending the command holds the aim it was planned for.
Track change or timeout resets the lock and panel hold.

Tuned values (`AimPlannerConstants.hpp`, moved unchanged from the Jetson defaults):

| Value | Setting |
|---|---|
| replan period | 25 ms (the Jetson's 40 Hz tick) |
| gimbal lag | 0.05 s |
| firmware latency / min fire delay | 20 ms (`FIRING_LATENCY_TIME`) / 0.040 s |
| lead, ToF iterations | on, 3 |
| spin mode | shotgating (chase off, margin 0); enter 3.0 rad/s, exit 2.0 |
| panel switch margin | 0.3 rad |
| muzzle speed | `initialShotVelocity` |
| fire confidence | 0.5 (`confidence / 255`) |
| fire lock / grace | 1.0 s / 0.3 s; stationary bypass below 0.1 m/s outside spin |
| max detection age to fire / target timeout | 0.15 s / 0.5 s |

The fire lock counts milliseconds on the MCB clock (wrap-safe), not the Jetson's node clock.
The orange debug circle shows for 100 ms after a shot is scheduled.

Absolute aim points use a filtered angular-rate estimate over the 1 ms
controller ticks: a 40 Hz point update must not become a one-tick velocity
impulse. The estimator wraps yaw differences, treats jumps above 0.25 rad as
target handoffs, and resets when leaving absolute aiming. Manual incremental
commands retain their rate semantics.

The gimbal controller uses the main scheduler's 1 ms interval. Its 8 ms yaw
predictor integrates geared output torque and the configured viscous/static
friction; the torque history starts at zero. Simulator sensor age must match
this predictor. The sentry ballistic solver assumes 22 m/s, the shot speed the team set
from the robot (2026-10-28); Jetson and sim use the same value. Calibration evidence lives in the
[sim accuracy investigation](../../sim/docs/aim-accuracy-2026-10-12.md).

The sentry and infantry solver (same gimbal, shared constants) uses CAD pivot height and bore length (`Projections.hpp`), horizontal
quadratic drag, and `SHOT_PITCH_DOWN_BIAS`, the angle shots leave below the solved pitch.
Pitch encoder offset or ratio errors cancel because the camera rides the pitch head, so a
miss that grows with range is boresight or flywheel lift: refit the bias from misses at two
ranges, never by moving the pivot height (a height change only shifts where the miss is zero).

## GCC compatibility

### Hosted hardware fixture

The hosted hardware pipe emits only the four-byte `MCB3` header before the
first tick output; it supplies no start XY. It carries ten DJI CAN feedback packets and
returns raw motor command words. Gimbal encoders and velocities use the same
Taproot decoder as the board; only the hosted yaw home is fixed to the sim
joint's boot zero. Sim applies motor forces inside gz's 1 ms step; its
[MCB emulator](../../sim/README.md#mcb-emulator) owns the plant and its limits.
Sentry pitch selects the calibrated angle nearest zero from the encoder's
single-turn reading. Its travel stays within half a motor revolution around
that zero, including boots on either side of the encoder wrap.
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
mappings, and the shared DJI UART baud constant. Preserve these fixes when regenerating from the pinned upstream
submodules. The hosted C++ regression tests check iterator traits, null command
filtering, UART payload bounds, and clock rollover.
