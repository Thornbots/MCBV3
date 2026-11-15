# Managed aiming validation, 2026-11-12

The [implemented design](../README.md#target-handoff-redesign) replaces the old
fire lock and held shot plan. This evidence covers hosted execution; Type C
scheduler acceptance remains open. Integrate matching Jetson packages and
reflash firmware together.

## Build and correctness

Linux arm64 in the task-owned Mac Colima container, capped at four CPUs, GCC 14,
release sentry hosted firmware:

```sh
CXX=g++-14 tools/test_cpp.sh
cd MCB-project
scons build-sim robot=sentry profile=release compiler-suffix=-14 -j4
cd ..
CXX=g++-14 tools/test_hosted_cpp.sh
```

- Pure C++: **76/76**, including 21 ballistic/planner/lifecycle tests;
  mapping isolation and CAN decoder Python checks: **7/7**.
- Actual hosted subsystems: **49/49**; Python pipe handshake: **3/3**.
- Optional `aim-timing=true` hosted build: passed (DWT reads zero on hosted).
- Current UART fixture: **61/64**; three unchanged nightly `McbPitchBoot`
  parameter cases fail. The production motor adapter and those tests are unchanged.
  New alignment tests supply explicit measured feedback and isolate the known
  fixture/firmware pitch-offset mismatch in their test setup.
- Package quality: zero new diagnostics (33 existing); actionlint passed.

The 6,960-case numerical sweep spans 1–8 m in 0.25 m steps, five relative
heights, 16/22/25 m/s, zero/nonzero drag, forward/up bore offsets and pitch bias.
Every solve is within 2 mrad and 0.5 ms of the ten-step Reticle reference, with
height residual at most 0.5 mm against an independent converged double reference.
Analytic derivatives are checked against finite differences. Singular,
unreachable, nonfinite and exhausted solves fail closed.

Lifecycle tests cover WAITING/TRACKING, clear over valid/malformed geometry,
track handoff, same-track hysteresis, exact 600 ms receipt expiry and rollover,
invalid frames not extending the deadline, immediate refresh and 5 ms cadence.
Rate tests cover acceleration, spin, shooter motion and latency back-projection.
Gate tests cover window conjunction, latency/rate signs, moving tilted planes,
oblique edges/corners, exact inclusive outline bounds, next-float rejection,
travel-limited misses, back faces and degenerate intersections. Real UART tests
cover clear precedence, aligned fire versus measured misses, and diagnostic-only
confidence/detection age. Dedicated mailbox tests preserve clear/assignment order
through intervening idle UART traffic.

## Execution cost

`tools/bench_aim.cpp` times 10,000 moving/spinning plans and measured-shot gates
using `steady_clock`, after compilation and without a concurrent build. This is
host wall time, including scheduling noise, **not Type C execution cost**.

| Path | Mean | p99 | Maximum | Samples |
| --- | ---: | ---: | ---: | ---: |
| Replan | 0.133 µs | 0.209 µs | 7.250 µs | 10,000 |
| Measured-shot gate | 0.082 µs | 0.125 µs | 3.792 µs | 10,000 |

9,961 plans were valid and all 9,961 intersected inside the outline. The other
39 phase-boundary cases failed closed. The clock granularity and host scheduler
limit interpretation of sub-microsecond samples.

```sh
g++-14 -O3 -std=c++17 -Wall -Wextra -Werror -IMCB-project/src \
  tools/bench_aim.cpp MCB-project/src/subsystems/jetson/AimPlanner.cpp -o /tmp/bench-aim
/tmp/bench-aim
```

The static work bound is at most two fixed-panel candidates × four outer
residual evaluations × four ballistic samples = **32 ballistic height samples
per replan**. The gate has at most **four moving-plane evaluations**. Each scalar
Newton solve has at most three corrections and a final residual check. These
operation limits are not a cycle bound: target libm costs are unmeasured.

No STM32F407 cycles have been measured. Build the board with `aim-timing=true`
and inspect `aim::aim_timing` through a debugger. Record normal replans, panel
handoffs, exhaustion/failure paths, and gates with the rest of the scheduler
running. Confirm both path budgets from the design and no missed complete 1 ms
controller deadlines; the DWT path counters alone do not prove scheduler timing.
Hosted and Renode wall timing cannot close this acceptance item.

## Integration limits

The stand-in comparison, fixtures and scorer limitations are maintained in
[sim's dated results](../../../sim/docs/managed-aim-2026-11-12.md).
Final validation uses bridge commit
`2e9a435b709697f3fecb3f21034505daa4cf40eb`, including `bool clear` in
`TargetState.msg`; it does not rely on the earlier local message stub.

The coordinated contract accepts finite nonnegative radii in firmware and
requires positive radii in the manager. Malformed assignments retain the prior
accepted model without refreshing receipt expiry. A clear still wins over all
model fields and track id; same-ID reacquisition clears before reassigning.

## Nightly integration follow-up

Merged firmware nightly `8405b0d` without rewriting the PR history, retaining
the nearest-zero pitch decoding for every robot alongside the redesign.
The four-CPU task container passes all 76 pure C++ tests, 49 hosted C++ tests
and three hosted handshake tests after a fresh hosted sentry rebuild. The strict
sim UART fixture passes all 64 tests with sim `2fd2f940cc1fe37d593a6d0de3a0bbc876613e02`;
that immutable revision is now pinned by firmware CI. It includes sim nightly's
pitch-zero parity fix and removes the obsolete test-only pitch compensation.
Package quality and actionlint pass. No board timing was measured during this
follow-up; the execution-cost acceptance above remains open.
