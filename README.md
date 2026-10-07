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

All code is written in `MCB-Project/src/`. If you understand typical C++
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

This command will build the robot code, prepping it to flash, then will proceed to flash it using `openocd`. Alternatively, you can run `scons build` to just build the code. You'll notice that this code will default to compiling the standard project. You can specify the robot type and the system identificaton enable using the command below:

```
scons run robot=<ROBOT_TYPE> sysid=<SYSID_TYPE>
```

These arguments will work for `scons build` too!

## Continuous integration

GitHub Actions compiles ARM firmware for `infantry`, `hero`, and `sentry`
on every push and pull request. Each build uploads its ELF file for
inspection; CI never flashes a board. The hosted sentry job runs the package's
geometry and chassis GoogleTests, then builds real firmware and exercises its
UART parser, referee gates, aiming, firing, and driving through the pinned
`Thornbots/sim` fixture. A missing executable or skipped UART test fails CI.

Run the C++ tests in a Linux terminal with GCC 11 and GoogleTest installed:

```bash
tools/test_cpp.sh
```

This runner tests package-owned code directly. The historical `scons run-tests`
target also compiles vendored Taproot test suites and needs additional GoogleMock
dependencies. The UART fixture revision is pinned in
`.github/workflows/firmware.yml`; update it deliberately when the wire protocol
changes. Existing compiler warnings remain visible in CI, including deprecated
motor accessors and constructor member ordering.

The legacy `oldinfantry` target is excluded: its indexer homing offset and power
limiter torque scaling are missing. Restore measured calibration values before
enabling that target in CI.
