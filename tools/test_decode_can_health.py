#!/usr/bin/env python3
"""Host-only regression tests for the CAN health payload decoder."""
import struct
import unittest

import argparse

from decode_can_health import command_frames, decode


class CanHealthDecoderTest(unittest.TestCase):
    def test_rates_and_esr(self):
        header = struct.pack('<BIII', 1, 7, 2000, 2000)
        bus = struct.pack('<11IB', 100, 0, *([2000] * 8), 0x34120067, 1)
        text = decode(header + bus + bus)
        for expected in ('seq=7', '5.00%', '50.0/s', '1000.0/s', '100.0%',
                         'TEC=18 REC=52 LEC=6 EPVF=1 BOFF=1 EWGF=1',
                         'send_failures=unavailable'):
            self.assertIn(expected, text)

    def test_load_estimate(self):
        header = struct.pack('<BIII', 1, 0, 1000, 1000)
        can1 = struct.pack('<11IB', 0, 0, *([1000] * 4), *([0] * 4), 0, 1)
        can2 = struct.pack('<11IB', 500, 0, *([0] * 8), 0, 1)
        text = decode(header + can1 + can2, (1, 2))
        self.assertIn('load>=55.5-67.5% (feedback 4000/s + commands 1000/s', text)
        self.assertIn('load>=11.1-13.5% (feedback 0/s + commands 1000/s', text)
        self.assertNotIn('load', decode(header + can1 + can2))

    def test_command_frames_argument(self):
        self.assertEqual(command_frames('2,1'), (2, 1))
        for text in ('2', '1,-1', 'a,b', '1,2,3'):
            with self.assertRaises(argparse.ArgumentTypeError):
                command_frames(text)

    def test_hosted_fields_are_unavailable(self):
        payload = struct.pack('<BIII', 1, 0, 1000, 0) + bytes(90)
        self.assertIn('ESR: unavailable', decode(payload))
        self.assertIn('n/a', decode(payload))

    def test_invalid_layouts(self):
        for payload in (b'', bytes(103), bytes([2]) + bytes(102), bytes([1]) + bytes(102)):
            with self.assertRaises(ValueError):
                decode(payload)


if __name__ == '__main__':
    unittest.main()
