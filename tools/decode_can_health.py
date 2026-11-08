#!/usr/bin/env python3
"""Decode CAN health DEBUG blobs from raw payload hex or a ROS 2 bag."""

import argparse
import struct

# Standard 11-bit ID, 8-byte data frame incl. IFS: unstuffed / worst-case stuffed bits.
FRAME_BITS = (111, 135)
BITRATE = 1_000_000


def decode(data, command_frames=None):
    """Reject unrelated DEBUG layouts; return printable per-window indicators.

    command_frames: optional (CAN1, CAN2) command frames sent per ready cycle;
    enables a lower-bound load estimate (counted motor frames only).
    """
    if len(data) != 103 or data[0] != 1:
        raise ValueError('expected CAN health v1 (103 bytes)')
    sequence, milliseconds, cycles = struct.unpack_from('<III', data, 1)
    if milliseconds == 0:
        raise ValueError('zero-length reporting window')
    seconds = milliseconds / 1000
    lines = [f'seq={sequence} window={milliseconds}ms sampled_cycles={cycles}']
    for bus in range(2):
        offset = 13 + 45 * bus
        skipped, failures, *counts, esr, flags = struct.unpack_from('<11IB', data, offset)
        fraction = f'{100 * skipped / cycles:.2f}%' if cycles else 'n/a'
        lines.append(f'CAN{bus + 1}: not-ready={skipped} ({fraction} of sampled cycles, '
                     f'{skipped / seconds:.1f}/s); send_failures='
                     f'{failures if flags & 2 else "unavailable"}')
        for motor, count in enumerate(counts, 1):
            rate = count / seconds
            lines.append(f'  ID{motor} (0x{0x200 + motor:03x}): {rate:.1f}/s '
                         f'({100 * rate / 1000:.1f}% of 1000/s)')
        if command_frames is not None:
            feedback = sum(counts) / seconds
            commands = command_frames[bus] * max(cycles - skipped, 0) / seconds
            low, high = (100 * (feedback + commands) * bits / BITRATE for bits in FRAME_BITS)
            lines.append(f'  load>={low:.1f}-{high:.1f}% (feedback {feedback:.0f}/s + '
                         f'commands {commands:.0f}/s; excludes other nodes and error frames)')
        if flags & 1:
            lines.append(f'  ESR: TEC={(esr >> 16) & 255} REC={(esr >> 24) & 255} '
                         f'LEC={(esr >> 4) & 7} EPVF={(esr >> 1) & 1} '
                         f'BOFF={(esr >> 2) & 1} EWGF={esr & 1}')
        else:
            lines.append('  ESR: unavailable (hosted/sim)')
    return '\n'.join(lines)


def command_frames(text):
    """Parse 'CAN1,CAN2' non-negative per-cycle command frame counts."""
    try:
        counts = tuple(int(part) for part in text.split(','))
    except ValueError:
        counts = ()
    if len(counts) != 2 or min(counts) < 0:
        raise argparse.ArgumentTypeError('expected two non-negative integers, e.g. 2,1')
    return counts


def bag_payloads(path, topic):
    """Use the installed bridge message type and rosbag2 storage autodetection."""
    import rosbag2_py
    from dji_serial_bridge.msg import DebugBlob
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    while reader.has_next():
        _, serialized, _ = reader.read_next()
        yield bytes(deserialize_message(serialized, DebugBlob).data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--hex', help='hex of the DEBUG payload, without UART framing')
    source.add_argument('--bag', help='rosbag2 directory or storage file; source ROS/bridge first')
    parser.add_argument('--topic', default='/dji_serial_bridge/debug')
    parser.add_argument('--command-frames', type=command_frames, metavar='CAN1,CAN2',
                        help='command frames each bus sends per cycle (e.g. 2,1); '
                             'adds an estimated load range')
    args = parser.parse_args()
    if args.hex is not None:
        try:
            print(decode(bytes.fromhex(args.hex), args.command_frames))
        except ValueError as error:
            parser.error(str(error))
    else:
        for payload in bag_payloads(args.bag, args.topic):
            try:
                print(decode(payload, args.command_frames))
            except ValueError:
                continue  # DEBUG may also contain unrelated producers' blobs.


if __name__ == '__main__':
    main()
