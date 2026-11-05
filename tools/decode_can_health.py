#!/usr/bin/env python3
"""Decode CAN health DEBUG blobs from raw payload hex or a ROS 2 bag."""

import argparse
import struct


def decode(data):
    """Reject unrelated DEBUG layouts; return printable per-window indicators."""
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
        if flags & 1:
            lines.append(f'  ESR: TEC={(esr >> 16) & 255} REC={(esr >> 24) & 255} '
                         f'LEC={(esr >> 4) & 7} EPVF={(esr >> 1) & 1} '
                         f'BOFF={(esr >> 2) & 1} EWGF={esr & 1}')
        else:
            lines.append('  ESR: unavailable (hosted/sim)')
    return '\n'.join(lines)


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
    args = parser.parse_args()
    if args.hex is not None:
        try:
            print(decode(bytes.fromhex(args.hex)))
        except ValueError as error:
            parser.error(str(error))
    else:
        for payload in bag_payloads(args.bag, args.topic):
            try:
                print(decode(payload))
            except ValueError:
                continue  # DEBUG may also contain unrelated producers' blobs.


if __name__ == '__main__':
    main()
