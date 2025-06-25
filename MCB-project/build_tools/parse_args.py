# Copyright (c) 2020-2021 Thornbots
#
# This file is part of MCB.
#
# MCB is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# MCB is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with MCB.  If not, see <https://www.gnu.org/licenses/>.

from SCons.Script import *
from . import extract_robot_type


CMD_LINE_ARGS                       = 1
TEST_BUILD_TARGET_ACCEPTED_ARGS     = ["build-tests", "run-tests", "run-tests-gcov"]
SIM_BUILD_TARGET_ACCEPTED_ARGS      = ["build-sim", "run-sim"]
HARDWARE_BUILD_TARGET_ACCEPTED_ARGS = ["build", "run", "size", "gdb", "all"]
VALID_BUILD_PROFILES                = ["debug", "release", "fast"]
VALID_PROFILING_TYPES               = ["true", "false"]

#added by Alex Stedman for mappings ============================
SYS_ID_DEFINES                      = {"none": "no_sysid",
                                       "dt": "drivetrain_sysid",
                                       "yaw": "yaw_sysid",
                                       "odo": "odom_sysid"}

ROBOT_TYPE_DEFINES                  = {"oldinfantry": "OLDINFANTRY",
                                       "oldstandard": "OLDINFANTRY",
                                       "OLDINFANTRY": "OLDINFANTRY",
                                       "OLDSTANDARD": "OLDINFANTRY",
                                       "infantry": "INFANTRY",
                                       "standard": "INFANTRY",
                                       "INFANTRY": "INFANTRY",
                                       "STANDARD": "INFANTRY",
                                       "hero": "HERO",
                                       "HERO": "HERO",
                                       "sentry": "SENTRY",
                                       "SENTRY": "SENTRY"}

REF_MESSAGES_FAKING_DEFINES         = {"none": "regular_ref",
                                       "fake": "faked_ref",
                                       "faked": "faked_ref",
                                       "sim": "faked_ref",
                                       "simulate": "faked_ref",
                                       "simulated": "faked_ref",
                                       "lie": "faked_ref",
                                       "lying": "faked_ref"}
#===================================================================

USAGE = "Usage: scons <target> [profile=<debug|release|fast>] [robot=<infantry|hero|sentry>] [profiling=<true|false>] [sysid=<none|dt|yaw|odo>] [ref=<none|fake>]\n\
    \"<target>\" is one of:\n\
        - \"build\": build all code for the hardware platform.\n\
        - \"run\": build all code for the hardware platform, and deploy it to the board via a connected ST-Link.\n\
        - \"build-tests\": build core code and tests for the current host platform.\n\
        - \"run-tests\": build core code and tests for the current host platform, and execute them locally with the test runner.\n\
        - \"run-tests-gcov\": builds core code and tests, executes them locally, and captures and prints code coverage information\n\
        - \"build-sim\": build all code for the simulated environment, for the current host platform.\n\
        - \"run-sim\": build all code for the simulated environment, for the current host platform, and execute the simulator locally.\n\
    \"robot=<ROBOT_TYPE>\" is an optional argument that can override whatever robot type has been specified in robot_type.hpp.\n\
        - <ROBOT_TYPE> has a few aliases but will be one of the following:\n\
            - OLDINFANTRY, INFANTRY, HERO, SENTRY.\n\
        - \"sysid\" lets you run sysid stuff yay\n\
        - \"ref\" if fake, then after a certain amount of time the match 'starts,' for simulating connecting to a server for sentry"


def parse_args():
    args = {
        "TARGET_ENV": "",
        "BUILD_PROFILE": "",
        "PROFILING": "",
        "ROBOT_TYPE": "",
        "SYS_ID": "",
        "BUILD": "",
    }
    if len(COMMAND_LINE_TARGETS) > CMD_LINE_ARGS:
        raise Exception("You did not enter the correct number of arguments.\n" + USAGE)

    # Extract the target environment from the first command line argument
    # and determine modm build path as well as add any extra files to ignore
    if len(COMMAND_LINE_TARGETS) != 0:
        build_target = COMMAND_LINE_TARGETS[0]
        if build_target == "help":
            print(USAGE)
            exit(0)
        elif build_target in TEST_BUILD_TARGET_ACCEPTED_ARGS:
            args["TARGET_ENV"] = "tests"
        elif build_target in SIM_BUILD_TARGET_ACCEPTED_ARGS:
            args["TARGET_ENV"] = "sim"
        elif build_target in HARDWARE_BUILD_TARGET_ACCEPTED_ARGS:
            args["TARGET_ENV"] = "hardware"
        else:
            raise Exception("You did not select a valid target.\n" + USAGE)
    else:
        raise Exception("You must select a valid robot target.\n" + USAGE)

    # Extract and validate the build profile (either debug or release)
    args["BUILD_PROFILE"] = ARGUMENTS.get("profile", "release")
    if args["BUILD_PROFILE"] not in VALID_BUILD_PROFILES:
        raise Exception("You specified an invalid build profile.\n" + USAGE)
    
#=====================================ADDED BY Alex Stedman @Thornbots====================================

    sysid = ARGUMENTS.get("sysid")
    if sysid in SYS_ID_DEFINES:
        args["SYS_ID"] = SYS_ID_DEFINES[sysid]
        print( "======================================================================================")
        print(f"          !!!IMPORTANT!!!: Running SYS_ID on {SYS_ID_DEFINES[sysid]}")

    elif sysid != None:
        raise Exception("You specified an invalid sysid type.\n" + USAGE)
    else: #nothing defined so do none
        args["SYS_ID"] = SYS_ID_DEFINES["none"]


    ref = ARGUMENTS.get("ref")
    if ref in REF_MESSAGES_FAKING_DEFINES:
        args["REF"] = REF_MESSAGES_FAKING_DEFINES[ref]
        print( "======================================================================================")
        print(f"          !!!IMPORTANT!!!: Using ref system type {REF_MESSAGES_FAKING_DEFINES[ref]}")

    elif ref != None:
        raise Exception("You specified an invalid ref type.\n" + USAGE)
    else: #chose incorrectly so use regular ref
        args["REF"] = REF_MESSAGES_FAKING_DEFINES["none"]

    args["PROFILING"] = ARGUMENTS.get("profiling", "false")
    if args["PROFILING"] not in VALID_PROFILING_TYPES:
        raise Exception("You specified an invalid profiling type.\n" + USAGE)

    # Extract the robot type from either the command line or robot_type.hpp

    rtype = ARGUMENTS.get("robot")
    if rtype in ROBOT_TYPE_DEFINES:
        args["ROBOT_TYPE"] = ROBOT_TYPE_DEFINES[rtype]
    elif rtype != None:
        raise Exception(rtype+ " is an invalid robot type.\n" + USAGE)
    else: #nothing defined so do none
        print( "======================================================================================")
        print(f"          !!!WARNING!!!: No robot type was specified, defaulting to OLDINFANTRY          ")
        args["ROBOT_TYPE"] = ROBOT_TYPE_DEFINES["OLDINFANTRY"]

    print( "======================================================================================")
#=================================================================================================================
    return args
