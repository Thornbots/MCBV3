"""Exercise the hosted pipe boundary without the ROS fixture."""
import os
from pathlib import Path
import struct
import tty
import subprocess
import unittest


class HostedHandshake(unittest.TestCase):
    def test_header_is_immediately_followed_by_tick_output(self):
        binary = Path(__file__).resolve().parents[1] / (
            'MCB-project/build/sim/scons-release/MCB-project.elf')
        master, slave = os.openpty()
        try:
            # One stopped tick, no referee bytes, no CAN motion, gravity +z.
            tick = bytearray(168)
            struct.pack_into('<I', tick, 0, 1)
            struct.pack_into('<f', tick, 164, 1.0)
            result = subprocess.run(
                [str(binary)], input=tick, capture_output=True, timeout=10,
                env={**os.environ, 'MCB_UART_FD': str(master)}, pass_fds=(master,),
                check=True)
            self.assertEqual(result.stdout[:4], b'MCB3')
            self.assertEqual(len(result.stdout), 4 + 48)
            self.assertEqual(struct.unpack_from('<I', result.stdout, 4 + 24)[0], 1)
        finally:
            os.close(master)
            os.close(slave)

    def test_debug_flood_is_opt_in_and_carries_opaque_fixture_bytes(self):
        binary = Path(__file__).resolve().parents[1] / (
            'MCB-project/build/sim/scons-release/MCB-project.elf')
        for setting in ('', '0', 'true', '1'):
            with self.subTest(setting=setting):
                master, slave = os.openpty()
                try:
                    tty.setraw(slave)
                    os.set_blocking(slave, False)
                    tick = bytearray(168)
                    struct.pack_into('<I', tick, 0, 100)
                    struct.pack_into('<f', tick, 164, 1.0)
                    result = subprocess.run(
                        [str(binary)], input=tick * 3, capture_output=True, timeout=10,
                        env={**os.environ, 'MCB_UART_FD': str(master),
                             'MCB_DEBUG_TEST_FLOOD': setting},
                        pass_fds=(master,), check=True)
                    self.assertEqual(len(result.stdout), 4 + 3 * 48)
                    wire = bytearray()
                    while True:
                        try:
                            wire.extend(os.read(slave, 4096))
                        except BlockingIOError:
                            break
                    debug = []
                    types = []
                    offset = 0
                    while offset < len(wire):
                        self.assertEqual(wire[offset], 0xa5)
                        size = struct.unpack_from('<H', wire, offset + 1)[0]
                        kind = struct.unpack_from('<H', wire, offset + 5)[0]
                        types.append(kind)
                        end = offset + size + 9
                        self.assertLessEqual(end, len(wire))
                        if kind == 6:
                            debug.append(bytes(wire[offset + 7:end - 2]))
                        offset = end
                    self.assertEqual(types.count(2), 27)
                    self.assertEqual(types.count(3), 2)  # ticks cover 0..299 ms
                    expected = bytes([0, 0xa5, 0xff, 0x25]) + bytes(range(4, 128))
                    if setting == '1':
                        self.assertGreater(len(debug), 10)
                        self.assertTrue(all(blob == expected for blob in debug))
                    else:
                        self.assertEqual(debug, [])
                finally:
                    os.close(master)
                    os.close(slave)


if __name__ == '__main__':
    unittest.main(verbosity=2)
