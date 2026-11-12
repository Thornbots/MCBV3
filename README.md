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

<a id="target-handoff-redesign-proposed-review-before-implementation"></a>

#### Target handoff redesign

Implemented for `nightly` coordinated integration with matching Jetson
packages and firmware reflash. Board timing acceptance remains outstanding;
[validation evidence](docs/managed-aim-2026-11-12.md) separates hosted results
from Type C requirements. Jetson assignment/update/clear semantics belong to
the [TARGET_STATE contract](../../ros2_dji_serial_bridge/UART_PROTOCOL.md#proposed-target-assignment-update-and-clear);
the [target manager](../../thornbots_pkg/README.md#target_manager)
owns assignment policy. The bridge remains a translator.

The automatic aim command has two explicit states:

| State | Gimbal behavior | Firing | Transition |
| --- | --- | --- | --- |
| WAITING FOR TARGET | Existing idle CV_TARGET looks, firmware patrol and permitted hit turn | Never | An assigned model enters TRACKING AND SHOOTING |
| TRACKING AND SHOOTING | Replan every 5 ms; advance the setpoint with planner rates between replans | Shot window open **and** predicted impact inside the panel, with existing indexer/referee interlocks | Clear or link expiry enters WAITING FOR TARGET |

Startup and command termination reset to WAITING FOR TARGET. Assignment replaces
the held model immediately. A track-id change also resets panel selection, spin
hysteresis and aim-rate history; a same-track refresh preserves them. Clear and
link expiry discard the held model, cancel any unissued shot request and reset
planner state before the next firing decision. No shot request survives a target
change. A shot already physically committed to the indexer cannot be recalled.
Receipt-based link expiry and flag precedence are defined only in the wire
contract. Idle CV_TARGET messages cannot replace an assigned model.

**Continuous chase and rates.** The facing-panel chase behavior and panel-switch shot windows from
[PR #140](https://github.com/Thornbots/MCBV3/pull/140) remain the default. Replan
every **5 ms**, including while an issued shot is in flight. At 3–4 mm/ms, a target moves
15–20 mm between replans against a 135 mm panel; rate feedforward carries its
first-order motion between solves. This halves the unmodeled motion of a 10 ms
period while leaving five controller ticks per expensive solve. This replaces the
25 ms hold, its mid-hold timing term and frozen aim during pending requests. New assignments, clears and handoffs take effect
immediately rather than waiting for the next scheduled solve.
The center propagates with constant acceleration and armor with rigid spin from
the carried model stamp. Normal
refresh propagation is about 200 ms; link expiry bounds a stopped sender.

The planner returns yaw/pitch setpoints and their rates from the same
continuous intercept solution. These rates enter the absolute-angle
gimbal controller as feedforward, bypassing `AngleSetpointRate` for tracking.
Each intervening 1 ms controller tick uses `last_solved_setpoint + planner_rate
* elapsed_since_solve` as the requested setpoint.
Idle absolute points can retain their filtered derivative. Rates include shooter
motion, panel rotation and ballistic pitch in the same world-yaw, positive-down
pitch convention as measured feedback. A panel switch or track handoff uses the
new solution's rates without differentiating the jump; the measured-impact gate
checks acquisition. This removes `gimbal_lag_s`: the gate measures physical following
error. Hosted tests verify rate continuity; Type C timing acceptance remains pending.

**On target means impact anywhere on the whole panel.** Let `L` be the calibrated
request-to-muzzle latency (`FIRING_LATENCY_TIME`). Immediately before requesting
a shot, extrapolate measured yaw and pitch to muzzle exit as
`angle_exit = measured_angle + measured_rate * L`. Use physical gimbal rates in
the setpoint frame, not motor RPM or setpoint derivatives. Predict the bore
origin and shooter motion to that exit time, then launch the projectile from
these measured-angle predictions using the same drag/gravity/pitch-bias model
as the aim solve, including the same shooter-velocity compensation.

Find the projectile's forward intersection with the chosen panel's moving plane,
propagating the target's translation and spin to that intersection time. Let
`p`, `c`, `u` and `v` be the impact point, panel center, unit width tangent and
unit surface-height tangent at impact. The on-target test is:

```text
abs(dot(p - c, u)) <= 0.135 / 2
abs(dot(p - c, v)) <= 0.125 / 2
```

This is a test in the actual panel plane, not independent yaw/pitch error bands.
Include the complete outline, edges and corners, with no inset or dispersion
margin. The height is **125 mm along the surface at its 15° tilt**, the Reticle
comment's "height across the surface"; the 50 mm lit strip does not set the gate.
Construct the plane and tangents from the chosen panel's actual modeled normal
(yaw propagated to impact, with its 15° cant). Its angle to the shot and edge
depths therefore produce the correct foreshortening for an oblique panel. Do not
substitute a face-on rectangle or an inscribed angular box.

Keep the requested **unclamped** ballistic pitch as the planner's aim reference;
never replace it with the actuator's clamped command when assessing alignment.
The final gate uses the predicted shot from measured motion: there is no
`pitch_clamped` flag or separate veto. A travel-limited gimbal fails when its
predicted impact misses the outline; if the shot still hits an edge or corner,
take it. No valid forward intersection, a back-face/degenerate intersection or
a nonfinite solution means no shot.

Evaluate the chase shot window for a shot exiting at `now + L` and impacting at
`now + L + flight_time`. Fire only when that window and the on-target gate both
pass in the current 1 ms control cycle. Advance the last plan with its rates
between 5 ms replans, but evaluate the shot/window/outline gate from current
measurements before each request. Waiting for a future window must not latch
permission or freeze aiming: re-evaluate against the current model and measurements until
request time. Existing heat, referee, indexer and remote-control interlocks
remain prerequisites. The 1 s fire-lock dwell, grace window, confidence
gate, stationary bypass, detection-age gate, `target_timeout_s` and
`min_fire_delay_s` are removed. The link timeout replaces target expiry; measured alignment
replaces the minimum fire delay. Confidence belongs to Jetson assignment.

**One ballistic model for pitch and flight time.** `Ballistics.hpp` is a ROS/hardware-free
helper used by Reticle and AimPlanner. A successful solve returns
pitch and flight time together, using the same muzzle speed, bore origin,
pitch-down bias, gravity and horizontal quadratic drag as Reticle. For downrange
displacement `d` from the muzzle and initial horizontal speed `v_h`, its flight
time is `expm1(k*d)/(k*v_h)` (`d/v_h` when `k == 0`). Both `d` and `v_h` depend
on the solved pitch; center-to-center distance divided by muzzle speed is not a
valid substitute.

The bounded solve seeds the low-arc pitch with the gravity-only closed form,
then takes at most **three Newton corrections** of the existing horizontal-drag
height residual, with its analytic pitch derivative. Reuse sine/cosine,
`expm1` and derivative intermediates; do not finite-difference the residual or
nest Reticle's ten-step bisection inside each intercept iteration. Carry the
pitch-dependent bore origin and bias through each correction. Stop early on a
0.5 mm height residual; reject singular derivatives or failure to converge.
Same-track solves reuse the last converged pitch/time seeds. Implicit ballistic
and intercept derivatives provide rates without extra full solves.

Acceptance over **1–8 m** downrange, across reachable panel heights and the
configured muzzle speed, is at most **2 mrad pitch difference and 0.5 ms flight
time difference** from today's ten-step Reticle solve plus its own drag flight
time. The pitch tolerance allows the old bisection's roughly 1.53 mrad final
resolution; also require the new residual bound against a converged reference.
Include zero-drag, bore-offset and bias cases. Flight time and pitch must come
from the same solution, with no gravity-only time substituted after correction.

Solve the moving-panel intercept against that flight time, with shooter velocity
compensation and the target evaluated at muzzle exit plus flight time. Resolve
panel choice at impact consistently with chase. Use a bounded numerical solve
with an explicit residual tolerance and failure result; the tunable fixed
three-pass `tof_iterations` is removed. Cap the outer intercept correction at three steps,
require a flight-time residual at most 0.1 ms and fail closed on nonconvergence.
Keep panel identity fixed within each smooth solve and test neighboring
candidates at a chase boundary. No convergence means no shot.

Original Type C budget estimates (STM32F407 at 168 MHz, single-precision math,
optimized build): **0.1–0.3 ms per normal 5 ms replan**, about 17–50 thousand cycles,
with normal solves expected to converge before their hard limits; budget **0.5 ms** for
handoff/neighbor candidates. Budget a further **0.05–0.15 ms per 1 ms firing
check** for the measured-shot/moving-plane intersection, reusing the plan's
flight-time seed with at most three scalar root corrections. These are operation
budget estimates, not board measurements; libm and solver convergence dominate.
Acceptance requires on-board cycle measurements of both paths, worst-case panel
switches and iteration exhaustion, with the rest of the scheduler running and
no missed 1 ms controller deadline. Optimize or split bounded solve work across
ticks if necessary; hosted/Renode wall time cannot establish the board budget.

`AimPlannerTest` covers both states, clear precedence, track changes, refresh
preservation, exact receipt expiry/rollover, cadence/feedforward, window/outline
conjunction, latency/rate signs, moving/oblique panels, edges/corners,
travel-limited misses, and ballistic/intercept failures. Numerical sweeps compare
the ten-step Reticle and a converged reference. Bench and timing results live in
[the dated validation](docs/managed-aim-2026-11-12.md).

`AimControl` owns the held frame and has no pending shot queue. Its replan
solution describes muzzle exit; the controller back-projects its angles by
`rate * FIRING_LATENCY_TIME` to request the current angles. Advancing those
requests with the same rates then reaches the solved muzzle-exit direction,
without counting firing latency twice. The gate still extrapolates actual
measured motion independently. Shooter position is the current raw-odom pitch
pivot; propagation uses the model's carried age separately.

Each scalar Newton solve permits at most three corrections plus a final residual
check. A replan tests at most two fixed-panel candidates, each with four outer
residual evaluations and four inner ballistic evaluations: at most **32 height
samples**. A gate permits four moving-plane residual evaluations. These are
operation bounds, not cycle bounds. No claim of Type C deadline compliance is
made until DWT measurements with the scheduler running pass the budget.

Build with `aim-timing=true` to enable board DWT collection in
`aim::aim_timing`: control ticks, replans, panel handoffs, failed plans and
measured-shot gates each retain calls, maximum cycles and counts exceeding
168,000 cycles (1 ms at 168 MHz). Inspect through a debugger; this adds no UART
or ROS interface. Measure worst-case switches and iteration exhaustion as well
as normal tracking, and separately check the complete scheduler's deadline.
Hosted builds return zero DWT cycles and cannot satisfy this acceptance.

#### Controller integration

`AutoAimAndFireCommand` prioritizes an assigned model over idle CV_TARGET
(200 ms validity), hit turn and patrol. Invalid/unassigned frames do not invoke
a measured-panel fallback or extend the assignment link. Clear is accepted even
with unusable model fields and another track id. Malformed assigned geometry is
ignored without extending the link. A nonnegative radius is required. Dedicated
TARGET_STATE mailboxes retain a clear before a later assignment, even when idle
or other UART traffic arrives before the control tick. Confidence
and detection age are diagnostic only; the Jetson must enforce assignment sanity.

Source declarations in `AimPlannerConstants.hpp` own tuning. `Projections.hpp`
provides the robot's speed-independent bore/drag/bias calibration to both Reticle
and the command; `initialShotVelocity` supplies muzzle speed. The stand-in uses
the sentry defaults and a perfect gimbal; it is not a controller-plant test.
Tracking sends analytic yaw/pitch rates directly to the controllers; idle
absolute points retain `AngleSetpointRate`. The measured gate uses world-yaw
IMU rate and pitch encoder rate divided by gearing (plus base pitch motion on
infantry/hero), in the same frame as its angle. Requested pitch remains unclamped;
only the actuator command clamps it.

The existing yaw predictor and its sensor-age requirements remain unchanged.
Calibration evidence lives in the
[sim accuracy investigation](../../sim/docs/aim-accuracy-2026-10-12.md).
Refit a range-dependent boresight/flywheel miss through pitch bias at two ranges,
not by moving the pivot height.

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
When the MMM test opens its private control socket, `MappingStatus` additionally
reports actual navigation x/y, raw aim x/y and the navigation-localized flag.
These are measured after the control tick and let sim score held navigation
against truth, including rejected or dropped fixes. This hosted-only fixture
interface requires a matching sim checkout; it adds no UART telemetry.
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
