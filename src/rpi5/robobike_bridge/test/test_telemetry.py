"""ROS-independent checks for the /get_acc CSV parser and timing helpers."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robobike_bridge.telemetry import ClockMapper, GapDetector, parse_body, parse_line  # noqa: E402

LINE13 = "a,77904,0.000,0.000,36.000,0.000,4.087,-0.023,-0.058,-0.375,2.219,9.339,2.188"
LINE8 = "a,175715,47.000,24.435,80.000,-1.340,27.215,0.095"
MS = 1_000_000


class ParseLineTest(unittest.TestCase):
    def test_13_column_line(self):
        s = parse_line(LINE13)
        self.assertEqual((s.record_type, s.time_ms, s.columns), ("a", 77904, 13))
        self.assertEqual((s.sv_drv, s.sv_str, s.sv_std), (0.0, 0.0, 36.0))
        self.assertEqual((s.gy_roll, s.gy_yaw, s.gy_pitch), (-0.023, -0.058, -0.375))
        self.assertEqual((s.swp_sg, s.sv_pos), (0.0, 4.087))
        self.assertEqual((s.acc_x, s.acc_y, s.acc_z), (2.219, 9.339, 2.188))

    def test_8_column_line_has_nan_extensions(self):
        s = parse_line(LINE8)
        self.assertEqual((s.time_ms, s.columns), (175715, 8))
        self.assertEqual((s.sv_drv, s.sv_str, s.sv_std), (47.0, 24.435, 80.0))
        self.assertEqual((s.gy_roll, s.gy_yaw, s.gy_pitch), (-1.34, 27.215, 0.095))
        for name in ("swp_sg", "sv_pos", "acc_x", "acc_y", "acc_z"):
            self.assertTrue(math.isnan(getattr(s, name)), name)

    def test_crlf_and_spaces_are_tolerated(self):
        self.assertEqual(parse_line(" " + LINE8 + "\r").time_ms, 175715)

    def test_nan_and_inf_pass_through(self):
        s = parse_line("a,1,nan,inf,-inf,0,0,0")
        self.assertTrue(math.isnan(s.sv_drv))
        self.assertEqual((s.sv_str, s.sv_std), (math.inf, -math.inf))

    def test_invalid_lines_raise(self):
        for line in (
            "a,1,2,3",  # wrong column count
            "b,1,0,0,0,0,0,0",  # not a monitor record
            "a,x,0,0,0,0,0,0",  # non-integer time
            "a,-1,0,0,0,0,0,0",  # negative time
            "a,1,0,0,zz,0,0,0",  # non-numeric value
        ):
            with self.subTest(line=line), self.assertRaises(ValueError):
                parse_line(line)


class ParseBodyTest(unittest.TestCase):
    def test_mixed_body(self):
        body = "\n".join([LINE13, "", "garbage", LINE8, "a,1,2"]) + "\n"
        samples, malformed = parse_body(body)
        self.assertEqual([s.time_ms for s in samples], [77904, 175715])
        self.assertEqual(malformed, 2)

    def test_empty_body(self):
        self.assertEqual(parse_body(""), ([], 0))


class ClockMapperTest(unittest.TestCase):
    def test_samples_keep_4ms_spacing(self):
        mapper = ClockMapper(window_ns=10_000 * MS)
        mapper.observe(5_000 * MS, 1_000)
        self.assertEqual(mapper.stamp_ns(1_000) - mapper.stamp_ns(996), 4 * MS)
        self.assertEqual(mapper.stamp_ns(1_000), 5_000 * MS)

    def test_minimum_offset_is_used(self):
        mapper = ClockMapper(window_ns=10_000 * MS)
        mapper.observe(5_030 * MS, 1_000)  # 30 ms network delay
        mapper.observe(5_055 * MS, 1_050)  # 5 ms network delay -> smallest offset
        mapper.observe(5_140 * MS, 1_100)
        self.assertEqual(mapper.stamp_ns(1_100), 5_105 * MS)

    def test_old_observations_expire(self):
        mapper = ClockMapper(window_ns=1_000 * MS)
        mapper.observe(5_000 * MS, 1_000)
        mapper.observe(7_050 * MS, 3_000)
        self.assertEqual(mapper.stamp_ns(3_000), 7_050 * MS)

    def test_rewind_resets_estimate(self):
        mapper = ClockMapper(window_ns=10_000 * MS)
        mapper.observe(5_000 * MS, 100_000)
        self.assertTrue(mapper.observe(5_100 * MS, 50))
        self.assertEqual(mapper.rewinds, 1)
        self.assertEqual(mapper.stamp_ns(50), 5_100 * MS)

    def test_stamp_before_observation_is_none(self):
        self.assertIsNone(ClockMapper().stamp_ns(10))


class GapDetectorTest(unittest.TestCase):
    def test_counts_missing_samples(self):
        gaps = GapDetector()
        self.assertEqual([gaps.feed(t) for t in (100, 104, 108, 120, 124)], [0, 0, 0, 2, 0])
        self.assertEqual(gaps.missing, 2)

    def test_small_jitter_and_rewind_are_not_gaps(self):
        gaps = GapDetector()
        self.assertEqual([gaps.feed(t) for t in (100, 105, 111, 5, 9)], [0, 0, 0, 0, 0])


if __name__ == "__main__":
    unittest.main()
