# Standalone programming of the MCB via Openocd

## Prerequisites

- Download the latest version of [openocd](https://gnutoolchains.com/arm-eabi/openocd/). Download the newest version of the software and extract the files to a permanent location. The downloaded file is of type `.7zip`, so you will need a way to extract 7zip files, either by installing the 7zip extractor or by using an online unzipper.
- Add the path of the `/bin` directory of OpenOCD to your PATH environment variable.
    - In your Start menu, search for "environment" and select "Edit the system environment variables."

    - In the "User variables for _{your username here}_", where _{your username here}_ is a placeholder for your actual name. Double click on "Path."<br><br>
    <img src="https://gitlab.com/aruw/controls/aruw-mcb/uploads/c02e3c34b52f4c5756cdd9149e891da6/EnvironmentVarsMain.jpg" height="500px">

    - In "Edit environment variable," click "New."<br><br>
    <img src="https://gitlab.com/aruw/controls/aruw-mcb/uploads/44ba293d6876ca856ad51289c3bf2368/EnvironmentVarsPathVar.jpg" height="500px">

    - Add the absolute path of the `/bin` directory of OpenOCD to the box that shows up after you click "New."<br><br>
    <img src="https://gitlab.com/aruw/controls/aruw-mcb/uploads/8f25bbfc917288f9c92155f64b76d476/EnvironmentVarsPathAddition.jpg" height="500px">

    - Select "OK".
- Download the ST-Link V-2 driver [here](https://drive.google.com/drive/u/1/folders/1Ndk8Q-uUtzo3sQtzOoguDAVBZSM3IKT1), unzip, and run the executable to install the driver.

## How to deploy to the MCB

- Get an `.elf`: a [CI artifact](#ci-artifacts) or [your own build](#creating-an-elf-file).
- Get [`openocd.cfg`](openocd.cfg) from this directory; artifacts do not include it.
- In a terminal (bash or Windows), run:
    ```
    openocd -f path/to/openocd.cfg -c "modm_program path/to/MCB-project.elf"
    ```

## CI artifacts

GitHub Actions' "Firmware build and tests" workflow
([`firmware.yml`](../.github/workflows/firmware.yml)) uploads one release ELF
per robot and sysid mode, named `firmware-<robot>-<sysid>`, from pushes to
`main`/`nightly` and pull requests into `main`; see
[CI](../README.md#continuous-integration). Each artifact holds only
`MCB-project.elf`. Download it from the run's page under the repo's Actions
tab, or with the GitHub CLI:

```
gh run list -R Thornbots/MCBV3 -w firmware.yml -b nightly -e push -s success -L 1
gh run download <run-id> -R Thornbots/MCBV3 -n firmware-sentry-none
```

Check that the run's commit is the one you mean to flash.

## Creating an elf file

Set up and build as in [Writing and Flashing Code](../README.md#writing-and-flashing-code),
naming the robot explicitly:

```
cd MCB-project
scons build robot=sentry profile=release
```

The ELF is `MCB-project/build/hardware/scons-release/MCB-project.elf`
(`scons-<profile>` for other profiles).
