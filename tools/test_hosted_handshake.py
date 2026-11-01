"""Exercise the hosted pipe boundary without the ROS fixture."""
import os
from pathlib import Path
import struct
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


if __name__ == '__main__':
    unittest.main(verbosity=2)
