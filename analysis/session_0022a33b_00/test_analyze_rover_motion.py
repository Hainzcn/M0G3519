import struct
import unittest

import numpy as np

import analyze_rover_motion as analyzer


def make_frame(
    sequence: int,
    mcu_ms: int,
    button_sequence: int = 0,
    last_button: int = 0,
    active_button: int = 0,
    imu_accel: int = 100,
    left_rpm: int = 200,
    right_rpm: int = 210,
) -> bytes:
    prefix = struct.pack(
        "<2sBBBBBBHHIhhh",
        analyzer.MAGIC,
        1,
        0x83,
        analyzer.FRAME_SIZE,
        1,
        last_button,
        active_button,
        button_sequence,
        sequence,
        mcu_ms,
        imu_accel,
        left_rpm,
        right_rpm,
    )
    return prefix + struct.pack("<H", analyzer.crc16_ccitt_false(prefix))


class ParserTests(unittest.TestCase):
    def test_parser_recovers_after_noise_and_bad_crc(self) -> None:
        first = make_frame(10, 1000, button_sequence=3, last_button=3)
        bad = bytearray(make_frame(11, 1010))
        bad[16] ^= 0x40
        second = make_frame(12, 1020, button_sequence=4, last_button=4)

        data, stats = analyzer.parse_stream(b"noise" + first + bad + second)

        self.assertEqual(stats.frames, 2)
        self.assertEqual(stats.crc_errors, 1)
        self.assertEqual(stats.sequence_gaps, 1)
        np.testing.assert_array_equal(data["sequence"], [10, 12])
        np.testing.assert_array_equal(data["last_button"], [3, 4])

    def test_mode_inference_follows_menu_buttons(self) -> None:
        t = np.arange(1000, dtype=float) * 0.01
        regions = [(100, 200), (400, 500), (700, 800)]
        events = [
            (2.5, 1, 1),
            (5.5, 2, 2),
        ]

        segments = analyzer.assign_modes(regions, t, events, 3, None)

        self.assertEqual(
            [(segment.mode, segment.trial) for segment in segments],
            [(3, 1), (4, 1), (3, 2)],
        )

    def test_mcu_time_wrap_is_unwrapped(self) -> None:
        mcu_ms = np.asarray([0xFFFFFFF0, 0xFFFFFFFA, 4, 14], dtype=np.uint32)
        np.testing.assert_allclose(
            analyzer.unwrap_mcu_time(mcu_ms),
            [0.0, 0.01, 0.02, 0.03],
        )


if __name__ == "__main__":
    unittest.main()
