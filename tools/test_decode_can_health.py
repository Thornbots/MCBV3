#!/usr/bin/env python3
"""Host-only regression tests for the CAN health payload decoder."""
import struct
import unittest

from decode_can_health import decode


class CanHealthDecoderTest(unittest.TestCase):
    def test_rates_and_esr(self):
        header = struct.pack('<BIII', 1, 7, 2000, 2000)
        bus = struct.pack('<11IB', 100, 0, *([2000] * 8), 0x34120067, 1)
        text = decode(header + bus + bus)
        for expected in ('seq=7', '5.00%', '50.0/s', '1000.0/s', '100.0%',
                         'TEC=18 REC=52 LEC=6 EPVF=1 BOFF=1 EWGF=1',
                         'send_failures=unavailable'):
            self.assertIn(expected, text)

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
